#include "speed_controller.h"

#include <algorithm>

#include "check.h"

namespace {

constexpr float kPeriod = 0.02f;
constexpr MotorGains kGains = {
    .k_s = 0.012f, .k_v = 0.345f, .k_a = 0.0174f, .k_p = 0.277f, .k_i = 5.48f};

// The first motor's identified model (results/sysid/2026-11-09-run01),
// stepped finely within each control period. Returns the speed averaged over
// the period, as the encoder reading is.
class SimulatedMotor {
 public:
  float run_period(float power) {
    const int steps = 50;
    const double dt = kPeriod / steps;
    double distance = 0.0;
    for (int i = 0; i < steps; ++i) {
      const double direction = speed_ != 0.0 ? (speed_ > 0.0 ? 1.0 : -1.0)
                                             : (power > 0 ? 1.0 : -1.0);
      const double accel =
          (power - kGains.k_s * direction - kGains.k_v * speed_) / kGains.k_a;
      const double next = speed_ + accel * dt;
      distance += (speed_ + next) / 2.0 * dt;
      speed_ = next;
    }
    return static_cast<float>(distance / kPeriod);
  }

 private:
  double speed_ = 0.0;
};

void test_ramp_limits_each_step_to_max_accel() {
  SetpointRamp ramp(5.0f);
  const RampedSetpoint first = ramp.advance(1.0f, kPeriod);
  CHECK_NEAR(first.for_next_period, 0.05f, 1e-6);
  CHECK_NEAR(first.accel_for_next_period, 5.0f, 1e-4);
}

void test_reaches_target_without_overshoot() {
  SetpointRamp ramp(5.0f);
  SpeedController controller(kGains, 0.95f);
  SimulatedMotor motor;
  float measured = 0.0f;
  float peak = 0.0f;
  for (int i = 0; i < 100; ++i) {
    const RampedSetpoint setpoint = ramp.advance(1.0f, kPeriod);
    measured = motor.run_period(
        controller.power_for_next_period(setpoint, measured, kPeriod));
    peak = std::max(peak, measured);
  }
  CHECK_NEAR(measured, 1.0f, 0.005);
  CHECK(peak < 1.01f);
}

void test_brakes_once_stopped_at_both_moments() {
  SetpointRamp ramp(5.0f);
  SpeedController controller(kGains, 0.95f);
  for (int i = 0; i < 50; ++i) {
    controller.power_for_next_period(ramp.advance(1.0f, kPeriod), 1.0f,
                                     kPeriod);
  }
  float power = 1.0f;
  for (int i = 0; i < 20; ++i) {
    power = controller.power_for_next_period(ramp.advance(0.0f, kPeriod), 0.0f,
                                             kPeriod);
  }
  CHECK(power == 0.0f);
}

void test_output_never_exceeds_max_power() {
  SpeedController controller(kGains, 0.95f);
  const RampedSetpoint flat_out = {.at_last_reading = 2.5f,
                                   .for_next_period = 2.5f,
                                   .accel_for_next_period = 0.0f};
  float power = 0.0f;
  for (int i = 0; i < 100; ++i) {
    power = controller.power_for_next_period(flat_out, 0.0f, kPeriod);
  }
  CHECK(power <= 0.95f);
}

}  // namespace

int main() {
  test_ramp_limits_each_step_to_max_accel();
  test_reaches_target_without_overshoot();
  test_brakes_once_stopped_at_both_moments();
  test_output_never_exceeds_max_power();
  return report("speed_controller");
}
