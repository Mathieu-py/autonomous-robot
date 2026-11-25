#include "drive/drivetrain.h"

#include <gtest/gtest.h>

#include <limits>
#include <ostream>

// Shows the reason's name when a check fails.
inline void PrintTo(StopReason reason, std::ostream* os) {
  *os << stop_reason_name(reason);
}

namespace {

constexpr float kPeriod = 0.02f;
constexpr Wheel::Config kConfig = {
    .ticks_per_rev = 100.0f,
    .max_accel = 5.0f,
    .max_power = 0.95f,
    .gains =
        {.k_s = 0.01f, .k_v = 0.35f, .k_a = 0.017f, .k_p = 0.27f, .k_i = 5.5f},
};
constexpr Target kForward = {
    .is_open_loop_power = false, .first = 1.0f, .second = 1.0f};
constexpr Target kStop = {
    .is_open_loop_power = false, .first = 0.0f, .second = 0.0f};

Drivetrain make_drivetrain() {
  return Drivetrain(Wheel(kConfig, 0), Wheel(kConfig, 0));
}

Readings readings(int32_t first_count, int32_t second_count,
                  bool is_bumper_pressed = false) {
  return {.first_count = first_count,
          .second_count = second_count,
          .is_bumper_pressed = is_bumper_pressed,
          .battery_volts = 12.0f};
}

TEST(Drivetrain, MeasuresSpeedFromCounts) {
  Drivetrain drivetrain = make_drivetrain();
  // 2 ticks of 100 per rev in 0.02 s is 1 rev/s.
  const PeriodReport report =
      drivetrain.run_period(kForward, readings(2, -2), kPeriod);
  EXPECT_NEAR(report.first.speed_rps, 1.0f, 1e-4);
  EXPECT_NEAR(report.second.speed_rps, -1.0f, 1e-4);
}

TEST(Drivetrain, MeasuresSpeedAcrossACountWrap) {
  constexpr int32_t kNearMax = std::numeric_limits<int32_t>::max() - 1;
  Drivetrain drivetrain(Wheel(kConfig, kNearMax), Wheel(kConfig, 0));
  // max - 1 to min + 1 is 3 ticks forward.
  const PeriodReport report = drivetrain.run_period(
      kForward, readings(std::numeric_limits<int32_t>::min() + 1, 0), kPeriod);
  EXPECT_NEAR(report.first.speed_rps, 1.5f, 1e-4);
}

TEST(Drivetrain, RampsTowardASpeedTarget) {
  Drivetrain drivetrain = make_drivetrain();
  drivetrain.run_period(kForward, readings(0, 0), kPeriod);
  const PeriodReport report =
      drivetrain.run_period(kForward, readings(0, 0), kPeriod);
  // 5 rev/s² ramp: 0 then 0.1 rev/s, so 0.05 over the period just measured.
  EXPECT_EQ(report.stop_reason, StopReason::kNone);
  EXPECT_NEAR(report.setpoint_rps, 0.05f, 1e-6);
  EXPECT_GT(report.first.power, 0.0f);
  EXPECT_GT(report.second.power, 0.0f);
}

TEST(Drivetrain, PassesOpenLoopPowerThrough) {
  Drivetrain drivetrain = make_drivetrain();
  const Target power = {
      .is_open_loop_power = true, .first = 0.3f, .second = -0.2f};
  const PeriodReport report =
      drivetrain.run_period(power, readings(0, 0), kPeriod);
  EXPECT_NEAR(report.first.power, 0.3f, 1e-6);
  EXPECT_NEAR(report.second.power, -0.2f, 1e-6);
  EXPECT_NEAR(report.setpoint_rps, 0.0f, 1e-6);
}

TEST(Drivetrain, BumperStopsBothWheels) {
  Drivetrain drivetrain = make_drivetrain();
  drivetrain.run_period(kForward, readings(0, 0), kPeriod);
  const PeriodReport report =
      drivetrain.run_period(kForward, readings(0, 0, true), kPeriod);
  EXPECT_EQ(report.stop_reason, StopReason::kBumper);
  EXPECT_NEAR(report.first.power, 0.0f, 1e-6);
  EXPECT_NEAR(report.second.power, 0.0f, 1e-6);
}

TEST(Drivetrain, WheelsStayStillWhenAStopClears) {
  Drivetrain drivetrain = make_drivetrain();
  int32_t count = 0;
  for (int i = 0; i < 25; ++i) {
    count += 2;
    drivetrain.run_period(kForward, readings(count, count), kPeriod);
  }
  drivetrain.run_period(kForward, readings(count, count, true), kPeriod);

  // The ramp restarts from zero, not from the 1 rev/s before the stop.
  const PeriodReport report =
      drivetrain.run_period(kStop, readings(count, count), kPeriod);
  EXPECT_EQ(report.stop_reason, StopReason::kNone);
  EXPECT_NEAR(report.first.power, 0.0f, 1e-6);
  EXPECT_NEAR(report.second.power, 0.0f, 1e-6);
}

}  // namespace
