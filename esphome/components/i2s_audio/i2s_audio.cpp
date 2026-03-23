#include "i2s_audio.h"

#ifdef USE_ESP32

#include <algorithm>

#include <hal/dma_types.h>

namespace esphome {
namespace i2s_audio {

namespace {

// DMA descriptor payload size depends on the alignment rules of the target's DMA path.
uint32_t get_i2s_dma_buffer_size_limit() {
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  return DMA_DESCRIPTOR_BUFFER_MAX_SIZE_64B_ALIGNED;
#else
  return DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
#endif
}

uint32_t get_i2s_dma_active_slot_count(i2s_slot_mode_t slot_mode) { return slot_mode == I2S_SLOT_MODE_STEREO ? 2 : 1; }

size_t get_i2s_dma_bytes_per_sample(i2s_data_bit_width_t data_bit_width) {
#ifdef USE_ESP32_VARIANT_ESP32
  // Original ESP32 transfers samples in 16-bit units, so 8/16-bit audio still occupies 2 bytes
  // per sample in DMA memory and 24/32-bit audio occupies 4 bytes.
  return ((static_cast<uint32_t>(data_bit_width) + 15U) / 16U) * 2U;
#else
  return (static_cast<uint32_t>(data_bit_width) + 7U) / 8U;
#endif
}

size_t get_i2s_dma_bytes_per_frame(i2s_data_bit_width_t data_bit_width, i2s_slot_mode_t slot_mode) {
  return get_i2s_dma_bytes_per_sample(data_bit_width) * get_i2s_dma_active_slot_count(slot_mode);
}

I2SAudioDmaConfig get_legacy_i2s_dma_config(uint32_t sample_rate, i2s_data_bit_width_t data_bit_width,
                                            i2s_slot_mode_t slot_mode) {
  I2SAudioDmaConfig cfg;

  cfg.dma_desc_num = I2S_LEGACY_DMA_BUFFERS_COUNT;
  cfg.dma_frame_num = std::max<uint32_t>(
      1, static_cast<uint32_t>((static_cast<uint64_t>(I2S_LEGACY_DMA_BUFFER_DURATION_MS) * sample_rate) / 1000ULL));
  cfg.dma_buffer_size = static_cast<size_t>(cfg.dma_frame_num) * get_i2s_dma_bytes_per_frame(data_bit_width, slot_mode);
  cfg.dma_buffer_duration_ms = I2S_LEGACY_DMA_BUFFER_DURATION_MS;
  cfg.dma_buffers_duration_ms = I2S_LEGACY_DMA_BUFFER_DURATION_MS * I2S_LEGACY_DMA_BUFFERS_COUNT;

  return cfg;
}

}  // namespace

I2SAudioDmaConfig get_i2s_dma_config(bool hires_audio, uint32_t sample_rate, i2s_data_bit_width_t data_bit_width,
                                     i2s_slot_mode_t slot_mode) {
  if (!hires_audio) {
    return get_legacy_i2s_dma_config(sample_rate, data_bit_width, slot_mode);
  }

  I2SAudioDmaConfig cfg;

  const size_t bytes_per_frame = get_i2s_dma_bytes_per_frame(data_bit_width, slot_mode);
  if (bytes_per_frame == 0) {
    return cfg;
  }

  uint32_t dma_frame_num = std::max<uint32_t>(
      1, static_cast<uint32_t>((static_cast<uint64_t>(I2S_DMA_BUFFER_TARGET_DURATION_MS) * sample_rate) / 1000ULL));
  const uint32_t max_frame_num = get_i2s_dma_buffer_size_limit() / bytes_per_frame;
  dma_frame_num = std::min(dma_frame_num, max_frame_num);

  // ESP-IDF expects 24-bit buffers to stay aligned to whole 3-byte sample groups.
  if (data_bit_width == I2S_DATA_BIT_WIDTH_24BIT && dma_frame_num >= 3) {
    dma_frame_num -= dma_frame_num % 3;
  }

  dma_frame_num = std::max<uint32_t>(1, dma_frame_num);

  const uint32_t dma_buffer_duration_ms = std::max<uint32_t>(
      1, static_cast<uint32_t>(((static_cast<uint64_t>(dma_frame_num) * 1000ULL) + sample_rate - 1) / sample_rate));
  const uint32_t target_total_frames = std::max<uint32_t>(
      1, static_cast<uint32_t>(((static_cast<uint64_t>(I2S_DMA_BUFFERS_TARGET_DURATION_MS) * sample_rate) + 999ULL) /
                               1000ULL));
  // Once a single descriptor is clamped, expand the descriptor count to recover roughly 60 ms of
  // total DMA headroom without letting the queue grow unbounded.
  const uint32_t dma_desc_num = std::min<uint32_t>(
      I2S_DMA_BUFFERS_MAX_COUNT,
      std::max<uint32_t>(I2S_DMA_BUFFERS_MIN_COUNT, (target_total_frames + dma_frame_num - 1) / dma_frame_num));

  cfg.dma_desc_num = dma_desc_num;
  cfg.dma_frame_num = dma_frame_num;
  cfg.dma_buffer_size = static_cast<size_t>(dma_frame_num) * bytes_per_frame;
  cfg.dma_buffer_duration_ms = dma_buffer_duration_ms;
  cfg.dma_buffers_duration_ms = std::max<uint32_t>(
      1, static_cast<uint32_t>(((static_cast<uint64_t>(dma_desc_num) * dma_frame_num * 1000ULL) + sample_rate - 1) /
                               sample_rate));

  return cfg;
}

uint32_t get_i2s_mclk_hz(uint32_t sample_rate, i2s_mclk_multiple_t mclk_multiple) {
  return sample_rate * static_cast<uint32_t>(mclk_multiple);
}

bool i2s_apll_supported() {
#if SOC_I2S_SUPPORTS_APLL
  return true;
#else
  return false;
#endif
}

i2s_clock_src_t get_i2s_clock_source(bool configured_use_apll) {
  i2s_clock_src_t clk_src = I2S_CLK_SRC_DEFAULT;
#if SOC_I2S_SUPPORTS_APLL
  if (configured_use_apll) {
    clk_src = I2S_CLK_SRC_APLL;
  }
#else
  (void) configured_use_apll;
#endif
  return clk_src;
}

const char *i2s_clock_source_to_string(i2s_clock_src_t clk_src) {
#if SOC_I2S_SUPPORTS_APLL
  if (clk_src == I2S_CLK_SRC_APLL) {
    return "APLL";
  }
#endif
#if SOC_I2S_SUPPORTS_XTAL
  if (clk_src == I2S_CLK_SRC_XTAL) {
    return "XTAL";
  }
#endif
  return clk_src == I2S_CLK_SRC_DEFAULT ? "default" : "custom";
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
