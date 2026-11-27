#include "drive/drivetrain.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "stop_reason_printer.h"

namespace {

constexpr float kPeriod = 0.02f;
constexpr Wheel::Config kConfig = {
    .ticks_per_rev = 100.0f,
    .max_accel_rps2 = 5.0f,
    .max_power = 0.95f,
    .gains =
        {.k_s = 0.01f, .k_v = 0.35f, .k_a = 0.017f, .k_p = 0.27f, .k_i = 5.5f},
};
constexpr Target kForward = {
    .is_open_loop_power = false, .first = 1.0f, .second = 1.0f};
constexpr Target kStop = {
    .is_open_loop_power = false, .first = 0.0f, .second = 0.0f};
constexpr Target kTimedOut = {.is_open_loop_power = false,
                              .first = 0.0f,
                              .second = 0.0f,
                              .is_commanded = false};

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

TEST(Drivetrain, ArcKeepsItsCurveWhileRamping) {
  Drivetrain drivetrain = make_drivetrain();
  const Target arc = {
      .is_open_loop_power = false, .first = 0.5f, .second = 1.5f};
  drivetrain.run_period(arc, readings(0, 0), kPeriod);
  const PeriodReport report =
      drivetrain.run_period(arc, readings(0, 0), kPeriod);
  // The second wheel's 0.1 rev/s step is a 15th of its way, so the first
  // wheel also goes a 15th: 0.0333, or 0.0167 over the period just measured.
  EXPECT_NEAR(report.setpoint_rps, 0.5f / 15.0f / 2.0f, 1e-6);
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

TEST(Drivetrain, TimedOutCommandDoesNotClearAStop) {
  Drivetrain drivetrain = make_drivetrain();
  drivetrain.run_period(kForward, readings(0, 0, true), kPeriod);
  EXPECT_EQ(
      drivetrain.run_period(kTimedOut, readings(0, 0), kPeriod).stop_reason,
      StopReason::kBumper);
  EXPECT_EQ(drivetrain.run_period(kStop, readings(0, 0), kPeriod).stop_reason,
            StopReason::kNone);
}

// The old check missed a wheel still creeping under full power.
TEST(Drivetrain, CreepingWheelUnderFullPowerStops) {
  Drivetrain drivetrain = make_drivetrain();
  const Target first_only = {
      .is_open_loop_power = false, .first = 1.0f, .second = 0.0f};
  PeriodReport report{};
  int32_t count = 0;
  for (int i = 0; i < 75; ++i) {
    count += 1;  // 0.5 rev/s, held back by hand
    report = drivetrain.run_period(first_only, readings(count, 0), kPeriod);
  }
  EXPECT_EQ(report.stop_reason, StopReason::kFirstStall);
  EXPECT_NEAR(report.first.power, 0.0f, 1e-6);
}

TEST(Drivetrain, HeldWheelStopsWhileReversing) {
  Drivetrain drivetrain = make_drivetrain();
  const Target second_backward = {
      .is_open_loop_power = false, .first = 0.0f, .second = -1.0f};
  PeriodReport report{};
  for (int i = 0; i < 50; ++i) {
    report = drivetrain.run_period(second_backward, readings(0, 0), kPeriod);
  }
  EXPECT_EQ(report.stop_reason, StopReason::kSecondStall);
  EXPECT_NEAR(report.second.power, 0.0f, 1e-6);
}

// A floor-loaded wheel at the low-battery cutoff, as a first-order model.
class LoadedWheel {
 public:
  int32_t run_period(float power) {
    constexpr int kSteps = 20;
    constexpr double kDt = kPeriod / kSteps;
    for (int i = 0; i < kSteps; ++i) {
      const double drive = power * kBatteryShare - kConfig.gains.k_v * speed_;
      const bool is_held_by_friction =
          speed_ == 0.0 && std::fabs(drive) <= kFriction;
      if (is_held_by_friction) {
        continue;
      }
      const double friction =
          std::copysign(kFriction, speed_ != 0.0 ? speed_ : drive);
      double next =
          speed_ + (drive - friction) / (kConfig.gains.k_a * kInertia) * kDt;
      if (next * speed_ < 0.0) {
        next = 0.0;  // friction stops the wheel, it doesn't reverse it
      }
      revs_ += (speed_ + next) / 2.0 * kDt;
      speed_ = next;
    }
    return static_cast<int32_t>(std::floor(revs_ * kTicksPerRev));
  }

  static constexpr float kTicksPerRev = 2473.0f;

 private:
  static constexpr double kBatteryShare = 10.5 / 12.0;
  static constexpr double kInertia = 1.5;
  static constexpr double kFriction = 0.03;
  double speed_ = 0.0;
  double revs_ = 0.0;
};

TEST(Drivetrain, LoadedWheelsAtLowBatteryNeverStall) {
  Wheel::Config config = kConfig;
  config.ticks_per_rev = LoadedWheel::kTicksPerRev;
  Drivetrain drivetrain(Wheel(config, 0), Wheel(config, 0));
  LoadedWheel first;
  LoadedWheel second;
  int32_t first_count = 0;
  int32_t second_count = 0;
  float worst_spare_power = 0.0f;
  struct Leg {
    float target_rps;
    int periods;
  };
  for (const Leg leg : {Leg{2.5f, 75}, Leg{-2.5f, 100}, Leg{0.0f, 50}}) {
    const Target target = {.is_open_loop_power = false,
                           .first = leg.target_rps,
                           .second = leg.target_rps};
    for (int i = 0; i < leg.periods; ++i) {
      const PeriodReport report = drivetrain.run_period(
          target, readings(first_count, second_count), kPeriod);
      ASSERT_EQ(report.stop_reason, StopReason::kNone);
      const float spare_power = std::fabs(
          report.first.power - kConfig.gains.k_v * report.first.speed_rps);
      worst_spare_power = std::max(worst_spare_power, spare_power);
      first_count = first.run_period(report.first.power);
      second_count = second.run_period(report.second.power);
    }
  }
  EXPECT_LT(worst_spare_power, 0.45f);
}

TEST(Drivetrain, TimedOutCommandRampsTheWheelsDown) {
  Wheel::Config config = kConfig;
  config.ticks_per_rev = LoadedWheel::kTicksPerRev;
  Drivetrain drivetrain(Wheel(config, 0), Wheel(config, 0));
  LoadedWheel first;
  LoadedWheel second;
  PeriodReport report{};
  const auto run_period = [&](const Target& target) {
    const Readings counts = readings(first.run_period(report.first.power),
                                     second.run_period(report.second.power));
    report = drivetrain.run_period(target, counts, kPeriod);
  };
  for (int i = 0; i < 50; ++i) {
    run_period(kForward);
  }
  run_period(kTimedOut);
  run_period(kTimedOut);
  // Setpoints 1.0 then 0.9: 0.95 over the period just measured, not a stop.
  EXPECT_EQ(report.stop_reason, StopReason::kNone);
  EXPECT_NEAR(report.setpoint_rps, 0.95f, 1e-5);
  EXPECT_GT(report.first.power, 0.0f);
}

}  // namespace
