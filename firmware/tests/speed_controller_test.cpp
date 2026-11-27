#include "drive/speed_controller.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

constexpr float kPeriod = 0.02f;
constexpr MotorGains kGains = {
    .k_s = 0.012f, .k_v = 0.345f, .k_a = 0.0174f, .k_p = 0.277f, .k_i = 5.48f};

// Simulates the first motor's model (sysid run01): mean speed per period.
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

TEST(SetpointRamp, LimitsEachStepToMaxAccel) {
  SetpointRamp ramp(5.0f);
  const RampedSetpoint first = ramp.advance(1.0f, kPeriod);
  EXPECT_NEAR(first.for_next_period, 0.05f, 1e-6);
  EXPECT_NEAR(first.accel_for_next_period, 5.0f, 1e-4);
}

TEST(SetpointRamp, AllowsTheShareOfAStepOnePeriodReaches) {
  const SetpointRamp ramp(5.0f);
  EXPECT_NEAR(ramp.allowed_share(-1.0f, kPeriod), 0.1f, 1e-6);
  EXPECT_EQ(ramp.allowed_share(0.05f, kPeriod), 1.0f);
}

TEST(SetpointRamp, PartwayIsExactAtAFullShare) {
  SetpointRamp ramp(5.0f);
  ramp.advance(0.1f, kPeriod);
  EXPECT_EQ(ramp.partway_to(0.0f, 1.0f), 0.0f);
  EXPECT_NEAR(ramp.partway_to(0.3f, 0.5f), 0.2f, 1e-6);
}

TEST(SpeedController, ReachesTargetWithoutOvershoot) {
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
  EXPECT_NEAR(measured, 1.0f, 0.005);
  EXPECT_LT(peak, 1.01f);
}

TEST(SpeedController, BrakesOnceStoppedAtBothMoments) {
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
  EXPECT_EQ(power, 0.0f);
}

TEST(SpeedController, IntegralDoesNotWindUpWhileSaturated) {
  for (const float direction : {1.0f, -1.0f}) {
    SpeedController controller(kGains, 0.95f);
    const RampedSetpoint flat_out = {.at_last_reading = 2.5f * direction,
                                     .for_next_period = 2.5f * direction,
                                     .accel_for_next_period = 0.0f};
    float power = 0.0f;
    for (int i = 0; i < 100; ++i) {
      power = controller.power_for_next_period(flat_out, 0.0f, kPeriod);
    }
    EXPECT_NEAR(power, 0.95f * direction, 1e-6);

    // Once at speed, a wound-up integral would hold the output saturated.
    const float at_speed =
        controller.power_for_next_period(flat_out, 2.5f * direction, kPeriod);
    EXPECT_LT(std::fabs(at_speed), 0.95f);
  }
}

}  // namespace
