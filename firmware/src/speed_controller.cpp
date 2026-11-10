#include "speed_controller.h"

#include <algorithm>
#include <cmath>

namespace {

float sign_of(float value) {
  return value > 0.0f ? 1.0f : value < 0.0f ? -1.0f : 0.0f;
}

}  // namespace

RampedSetpoint SetpointRamp::advance(float target_rps, float period_seconds) {
  const float max_step = max_accel_ * period_seconds;
  const float step =
      std::clamp(target_rps - setpoint_rps_, -max_step, max_step);

  RampedSetpoint setpoint;
  setpoint.at_last_reading = (previous_setpoint_rps_ + setpoint_rps_) / 2.0f;
  setpoint.for_next_period = setpoint_rps_ + step / 2.0f;
  setpoint.accel_for_next_period = step / period_seconds;

  previous_setpoint_rps_ = setpoint_rps_;
  setpoint_rps_ += step;
  return setpoint;
}

float SpeedController::power_for_next_period(const RampedSetpoint& setpoint,
                                             float measured_rps,
                                             float period_seconds) {
  const bool is_stopped =
      setpoint.at_last_reading == 0.0f && setpoint.for_next_period == 0.0f;
  if (is_stopped) {
    integral_ = 0.0f;
    return 0.0f;
  }

  const float error = setpoint.at_last_reading - measured_rps;
  const float new_integral = integral_ + error * period_seconds;
  const float feedforward = gains_.k_s * sign_of(setpoint.for_next_period) +
                            gains_.k_v * setpoint.for_next_period +
                            gains_.k_a * setpoint.accel_for_next_period;
  const float power =
      feedforward + gains_.k_p * error + gains_.k_i * new_integral;

  // Anti-windup: while saturated, keep only an integral that pulls the power
  // back toward the limit (error opposite in sign to the power).
  const bool is_saturated = std::fabs(power) >= max_power_;
  const bool is_unwinding = (error > 0.0f) != (power > 0.0f);
  if (!is_saturated || is_unwinding) {
    integral_ = new_integral;
  }

  return std::clamp(power, -max_power_, max_power_);
}
