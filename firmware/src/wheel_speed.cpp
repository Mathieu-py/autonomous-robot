#include "wheel_speed.h"

WheelSpeed::WheelSpeed(float ticks_per_rev, int32_t count, absolute_time_t now)
    : ticks_per_rev_(ticks_per_rev), last_count_(count), last_time_(now) {}

void WheelSpeed::update(int32_t count, absolute_time_t now) {
  const int32_t ticks = count - last_count_;
  const float seconds =
      static_cast<float>(absolute_time_diff_us(last_time_, now)) / 1e6f;

  if (seconds > 0.0f) {  // guards against dividing by zero
    speed_rps_ = static_cast<float>(ticks) / seconds / ticks_per_rev_;
    last_period_seconds_ = seconds;
  }

  last_count_ = count;
  last_time_ = now;
}
