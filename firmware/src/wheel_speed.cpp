#include "wheel_speed.h"

void WheelSpeed::update(int32_t count, float period_seconds) {
  const int32_t ticks = count - last_count_;
  speed_rps_ = static_cast<float>(ticks) / period_seconds / ticks_per_rev_;
  last_count_ = count;
}
