#pragma once

#include <cstdint>
#include <optional>

#include "pico/time.h"

// Ticks every period_ms. If the loop falls behind it skips ahead rather than
// ticking back-to-back, since tiny gaps would make the speed readings jumpy.
class PeriodicTimer {
 public:
  PeriodicTimer(absolute_time_t start, uint32_t period_ms)
      : period_ms_(period_ms),
        last_tick_(start),
        next_tick_(delayed_by_ms(start, period_ms)) {}

  // When a tick is due, returns the seconds since the last one (always > 0).
  std::optional<float> tick(absolute_time_t now) {
    if (absolute_time_diff_us(next_tick_, now) < 0) {
      return std::nullopt;
    }
    const float seconds =
        static_cast<float>(absolute_time_diff_us(last_tick_, now)) / 1e6f;
    last_tick_ = now;
    next_tick_ = delayed_by_ms(next_tick_, period_ms_);
    if (absolute_time_diff_us(next_tick_, now) >= 0) {
      next_tick_ = delayed_by_ms(now, period_ms_);
    }
    return seconds;
  }

 private:
  uint32_t period_ms_;
  absolute_time_t last_tick_;
  absolute_time_t next_tick_;
};
