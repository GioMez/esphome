#pragma once

#ifdef USE_ESP32

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include <esp_idf_version.h>
#include <driver/i2s_std.h>
#include <hal/dma_types.h>
#include <hal/i2s_ll.h>

namespace esphome {
namespace i2s_audio {

// Keep individual DMA chunks short for latency, but allow a wider total DMA window so hi-res
// streams can absorb scheduler jitter without periodically dropping frames.
static constexpr uint32_t I2S_LEGACY_DMA_BUFFER_DURATION_MS = 15;
static constexpr uint32_t I2S_LEGACY_DMA_BUFFERS_COUNT = 4;
static constexpr uint32_t I2S_DMA_BUFFER_TARGET_DURATION_MS = 15;
static constexpr uint32_t I2S_DMA_BUFFERS_TARGET_DURATION_MS = 60;
static constexpr uint32_t I2S_DMA_BUFFERS_MIN_COUNT = 4;
static constexpr uint32_t I2S_DMA_BUFFERS_MAX_COUNT = 12;

// Effective DMA geometry after clamping the requested layout to the limits of the current SoC/IDF.
struct I2SAudioDmaConfig {
  uint32_t dma_desc_num{I2S_DMA_BUFFERS_MIN_COUNT};
  uint32_t dma_frame_num{0};
  size_t dma_buffer_size{0};
  uint32_t dma_buffer_duration_ms{1};
  uint32_t dma_buffers_duration_ms{I2S_DMA_BUFFERS_MIN_COUNT};
};

// DMA descriptor payload size depends on the alignment rules of the target's DMA path.
inline uint32_t get_i2s_dma_buffer_size_limit() {
#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
  return DMA_DESCRIPTOR_BUFFER_MAX_SIZE_64B_ALIGNED;
#else
  return DMA_DESCRIPTOR_BUFFER_MAX_SIZE_4B_ALIGNED;
#endif
}

inline uint32_t get_i2s_dma_active_slot_count(i2s_slot_mode_t slot_mode) {
  return slot_mode == I2S_SLOT_MODE_STEREO ? 2 : 1;
}

inline size_t get_i2s_dma_bytes_per_sample(i2s_data_bit_width_t data_bit_width) {
#ifdef USE_ESP32_VARIANT_ESP32
  // Original ESP32 transfers samples in 16-bit units, so 24-bit audio still occupies 4 bytes.
  return ((static_cast<uint32_t>(data_bit_width) + 15U) / 16U) * 2U;
#else
  return (static_cast<uint32_t>(data_bit_width) + 7U) / 8U;
#endif
}

inline size_t get_i2s_dma_bytes_per_frame(i2s_data_bit_width_t data_bit_width, i2s_slot_mode_t slot_mode) {
  return get_i2s_dma_bytes_per_sample(data_bit_width) * get_i2s_dma_active_slot_count(slot_mode);
}

inline I2SAudioDmaConfig get_legacy_i2s_dma_config(uint32_t sample_rate, i2s_data_bit_width_t data_bit_width,
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

// Derive the real DMA layout that the driver can sustain. The nominal 15 ms buffer length may be
// clipped by descriptor-size limits, so the speaker task must use these effective values instead
// of assuming the requested geometry survived unchanged.
inline I2SAudioDmaConfig get_i2s_dma_config(bool hires_audio, uint32_t sample_rate, i2s_data_bit_width_t data_bit_width,
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

inline uint32_t get_i2s_mclk_hz(uint32_t sample_rate, i2s_mclk_multiple_t mclk_multiple) {
  return sample_rate * static_cast<uint32_t>(mclk_multiple);
}

// Mirrors the IDF clock feasibility check closely enough to decide when the default source is
// clearly too slow and we should jump straight to APLL.
inline bool i2s_default_clock_can_support_mclk(uint32_t sample_rate, i2s_mclk_multiple_t mclk_multiple) {
#ifdef I2S_LL_DEFAULT_CLK_FREQ
  return static_cast<uint64_t>(I2S_LL_DEFAULT_CLK_FREQ) >
         (static_cast<uint64_t>(get_i2s_mclk_hz(sample_rate, mclk_multiple)) * 2ULL);
#else
  (void) sample_rate;
  (void) mclk_multiple;
  return true;
#endif
}

// APLL stays opt-in unless the SoC's default source cannot satisfy the requested MCLK. That keeps
// common cases simple while still rescuing high-rate modes such as early ESP32-P4 revisions on XTAL.
inline bool should_use_i2s_apll(bool hires_audio, bool configured_use_apll, uint32_t sample_rate,
                                i2s_mclk_multiple_t mclk_multiple) {
#if SOC_I2S_SUPPORTS_APLL
  if (!hires_audio) {
    return configured_use_apll;
  }
  return configured_use_apll || !i2s_default_clock_can_support_mclk(sample_rate, mclk_multiple);
#else
  (void) hires_audio;
  (void) configured_use_apll;
  (void) sample_rate;
  (void) mclk_multiple;
  return false;
#endif
}

inline bool i2s_apll_supported() {
#if SOC_I2S_SUPPORTS_APLL
  return true;
#else
  return false;
#endif
}

// Keep source selection centralized so every I2S user applies the same policy.
inline i2s_clock_src_t get_i2s_clock_source(bool hires_audio, bool configured_use_apll, uint32_t sample_rate,
                                            i2s_mclk_multiple_t mclk_multiple) {
  i2s_clock_src_t clk_src = I2S_CLK_SRC_DEFAULT;
#if SOC_I2S_SUPPORTS_APLL
  if (should_use_i2s_apll(hires_audio, configured_use_apll, sample_rate, mclk_multiple)) {
    clk_src = I2S_CLK_SRC_APLL;
  }
#else
  (void) hires_audio;
  (void) configured_use_apll;
  (void) sample_rate;
  (void) mclk_multiple;
#endif
  return clk_src;
}

// Small helper for startup logs so we can see which clock path actually won.
inline const char *i2s_clock_source_to_string(i2s_clock_src_t clk_src) {
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

class I2SAudioComponent;

class I2SAudioBase : public Parented<I2SAudioComponent> {
 public:
  void set_i2s_role(i2s_role_t role) { this->i2s_role_ = role; }
  void set_slot_mode(i2s_slot_mode_t slot_mode) { this->slot_mode_ = slot_mode; }
  void set_std_slot_mask(i2s_std_slot_mask_t std_slot_mask) { this->std_slot_mask_ = std_slot_mask; }
  void set_slot_bit_width(i2s_slot_bit_width_t slot_bit_width) { this->slot_bit_width_ = slot_bit_width; }
  void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
  void set_use_apll(uint32_t use_apll) { this->use_apll_ = use_apll; }
  void set_hires_audio(bool hires_audio) { this->hires_audio_ = hires_audio; }
  void set_mclk_multiple(i2s_mclk_multiple_t mclk_multiple) { this->mclk_multiple_ = mclk_multiple; }

 protected:
  i2s_role_t i2s_role_{};
  i2s_slot_mode_t slot_mode_;
  i2s_std_slot_mask_t std_slot_mask_;
  i2s_slot_bit_width_t slot_bit_width_;
  uint32_t sample_rate_;
  bool use_apll_;
  bool hires_audio_{false};
  i2s_mclk_multiple_t mclk_multiple_;
};

class I2SAudioIn : public I2SAudioBase {};

class I2SAudioOut : public I2SAudioBase {};

class I2SAudioComponent : public Component {
 public:
  i2s_std_gpio_config_t get_pin_config() const {
    return {.mclk = (gpio_num_t) this->mclk_pin_,
            .bclk = (gpio_num_t) this->bclk_pin_,
            .ws = (gpio_num_t) this->lrclk_pin_,
            .dout = I2S_GPIO_UNUSED,  // add local ports
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            }};
  }

  void set_mclk_pin(int pin) { this->mclk_pin_ = pin; }
  void set_bclk_pin(int pin) { this->bclk_pin_ = pin; }
  void set_lrclk_pin(int pin) { this->lrclk_pin_ = pin; }
  void set_port(int port) { this->port_ = port; }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
  int get_port() const { return this->port_; }
#else
  i2s_port_t get_port() const { return static_cast<i2s_port_t>(this->port_); }
#endif

  void lock() { this->lock_.lock(); }
  bool try_lock() { return this->lock_.try_lock(); }
  void unlock() { this->lock_.unlock(); }

 protected:
  Mutex lock_;

  I2SAudioIn *audio_in_{nullptr};
  I2SAudioOut *audio_out_{nullptr};
  int mclk_pin_{I2S_GPIO_UNUSED};
  int bclk_pin_{I2S_GPIO_UNUSED};
  int lrclk_pin_;
  int port_{};
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
