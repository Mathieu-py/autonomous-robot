#pragma once

#include <cstdint>
#include <optional>

// Skips missed ticks rather than bunching them (jumpy speed readings).
class PeriodicTimer {
 public:
  PeriodicTimer(uint64_t start_us, uint32_t period_ms)
      : period_us_(uint64_t{period_ms} * 1000),
        last_tick_us_(start_us),
        next_tick_us_(start_us + period_us_) {}

  // When a tick is due, returns the seconds since the last one (always > 0).
  std::optional<float> tick(uint64_t now_us) {
    if (now_us < next_tick_us_) {
      return std::nullopt;
    }
    const float seconds = static_cast<float>(now_us - last_tick_us_) / 1e6f;
    last_tick_us_ = now_us;
    next_tick_us_ += period_us_;
    if (now_us >= next_tick_us_) {
      next_tick_us_ = now_us + period_us_;
    }
    return seconds;
  }

 private:
  uint64_t period_us_;
  uint64_t last_tick_us_;
  uint64_t next_tick_us_;
};
