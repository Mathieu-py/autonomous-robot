#pragma once

#include <cstdint>

#include "speed_controller.h"
#include "wheel_speed.h"

// One side of the drive's speed loop: its measured speed, setpoint ramp and
// controller. main.cpp passes the encoder count in and applies the power.
class Wheel {
 public:
  struct Config {
    float ticks_per_rev;
    float max_accel;  // rev/s²
    float max_power;
    MotorGains gains;
  };

  Wheel(const Config& config, int32_t count)
      : speed_(config.ticks_per_rev, count),
        ramp_(config.max_accel),
        controller_(config.gains, config.max_power) {}

  void measure(int32_t count, float period_seconds) {
    speed_.update(count, period_seconds);
  }

  // Returns the ramped setpoint the last reading is compared to.
  float follow(float target_rps, float period_seconds);

  void set_power(float power) { power_ = power; }

  // Stops at once, bypassing the ramp, and forgets past error.
  void stop();

  int32_t count() const { return speed_.count(); }
  float speed_rps() const { return speed_.speed_rps(); }
  float power() const { return power_; }

 private:
  WheelSpeed speed_;
  SetpointRamp ramp_;
  SpeedController controller_;
  float power_ = 0.0f;
};
