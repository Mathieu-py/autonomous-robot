#pragma once

// Wheel speed control: feedforward motor model plus PI feedback, on a ramped
// setpoint. Speeds in rev/s; power is a PWM fraction.

// A motor's measured model (k_s, k_v, k_a) and its PI gains, in power units.
struct MotorGains {
  float k_s;  // static friction
  float k_v;  // per rev/s
  float k_a;  // per rev/s²
  float k_p;  // per rev/s of error
  float k_i;  // per revolution of accumulated error
};

// The setpoint at the two moments the controller uses; mixing them leads
// ramps (PID runs 06 and 07 in results/README.md).
struct RampedSetpoint {
  float at_last_reading;        // rev/s, middle of the period just measured
  float for_next_period;        // rev/s, middle of the period about to run
  float accel_for_next_period;  // rev/s²
};

// Moves the setpoint toward the target by at most max_accel per second.
class SetpointRamp {
 public:
  explicit SetpointRamp(float max_accel) : max_accel_(max_accel) {}

  RampedSetpoint advance(float target_rps, float period_seconds);

  // For emergency stops: jumps straight to zero, bypassing the ramp.
  void reset() { setpoint_rps_ = previous_setpoint_rps_ = 0.0f; }

 private:
  float max_accel_;
  float setpoint_rps_ = 0.0f;
  float previous_setpoint_rps_ = 0.0f;
};

// One wheel's controller.
class SpeedController {
 public:
  SpeedController(const MotorGains& gains, float max_power)
      : gains_(gains), max_power_(max_power) {}

  // Brakes, and forgets past error, once the setpoint is zero at both moments.
  float power_for_next_period(const RampedSetpoint& setpoint,
                              float measured_rps, float period_seconds);

  void reset() { integral_ = 0.0f; }

 private:
  MotorGains gains_;
  float max_power_;
  float integral_ = 0.0f;  // error summed over time, in revolutions
};
