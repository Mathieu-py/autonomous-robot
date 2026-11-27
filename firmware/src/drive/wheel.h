#pragma once

#include <cstdint>

#include "drive/speed_controller.h"
#include "drive/wheel_state.h"

// One wheel's speed loop: measured speed, setpoint ramp and controller.
class Wheel {
 public:
  struct Config {
    float ticks_per_rev;
    float max_accel_rps2;
    float max_power;
    MotorGains gains;
  };

  Wheel(const Config& config, int32_t count)
      : ticks_per_rev_(config.ticks_per_rev),
        state_{.count = count, .speed_rps = 0.0f, .power = 0.0f},
        ramp_(config.max_accel_rps2),
        controller_(config.gains, config.max_power) {}

  // Speed from the encoder count, averaged over the period.
  void measure(int32_t count, float period_seconds);

  // The share of the step to target_rps that this period's ramp allows.
  float allowed_share(float target_rps, float period_seconds) const {
    return ramp_.allowed_share(target_rps, period_seconds);
  }

  // Ramps share of the way; returns the setpoint the last reading meets.
  float follow(float target_rps, float share, float period_seconds);

  void set_power(float power) { state_.power = power; }

  // Stops at once, bypassing the ramp, and forgets past error.
  void stop();

  // Power far from what its speed needs, either way: high motor current.
  bool is_stalling() const;

  const WheelState& state() const { return state_; }

 private:
  float ticks_per_rev_;
  WheelState state_;
  SetpointRamp ramp_;
  SpeedController controller_;
};
