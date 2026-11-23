#include "drive/wheel.h"

void Wheel::measure(int32_t count, float period_seconds) {
  const int32_t ticks = count - state_.count;
  state_.speed_rps =
      static_cast<float>(ticks) / period_seconds / ticks_per_rev_;
  state_.count = count;
}

float Wheel::follow(float target_rps, float period_seconds) {
  const RampedSetpoint setpoint = ramp_.advance(target_rps, period_seconds);
  state_.power = controller_.power_for_next_period(setpoint, state_.speed_rps,
                                                   period_seconds);
  return setpoint.at_last_reading;
}

void Wheel::stop() {
  ramp_.reset();
  controller_.reset();
  state_.power = 0.0f;
}
