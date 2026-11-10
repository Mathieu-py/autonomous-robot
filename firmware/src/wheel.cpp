#include "wheel.h"

float Wheel::follow(float target_rps, float period_seconds) {
  const RampedSetpoint setpoint = ramp_.advance(target_rps, period_seconds);
  power_ = controller_.power_for_next_period(setpoint, speed_.speed_rps(),
                                             period_seconds);
  return setpoint.at_last_reading;
}

void Wheel::stop() {
  ramp_.reset();
  controller_.reset();
  power_ = 0.0f;
}
