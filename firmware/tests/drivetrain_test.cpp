#include "drivetrain.h"

#include "check.h"

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

void test_measures_speed_from_counts() {
  Drivetrain drivetrain = make_drivetrain();
  // 2 ticks of 100 per rev in 0.02 s is 1 rev/s.
  const PeriodReport report =
      drivetrain.run_period(kForward, readings(2, -2), kPeriod);
  CHECK_NEAR(report.first_rps, 1.0f, 1e-4);
  CHECK_NEAR(report.second_rps, -1.0f, 1e-4);
}

void test_ramps_toward_a_speed_target() {
  Drivetrain drivetrain = make_drivetrain();
  drivetrain.run_period(kForward, readings(0, 0), kPeriod);
  const PeriodReport report =
      drivetrain.run_period(kForward, readings(0, 0), kPeriod);
  // 5 rev/s² ramp: 0 then 0.1 rev/s, so 0.05 over the period just measured.
  CHECK(report.stop_reason == StopReason::kNone);
  CHECK_NEAR(report.setpoint_rps, 0.05f, 1e-6);
  CHECK(report.first_power > 0.0f);
  CHECK(report.second_power > 0.0f);
}

void test_passes_open_loop_power_through() {
  Drivetrain drivetrain = make_drivetrain();
  const Target power = {
      .is_open_loop_power = true, .first = 0.3f, .second = -0.2f};
  const PeriodReport report =
      drivetrain.run_period(power, readings(0, 0), kPeriod);
  CHECK_NEAR(report.first_power, 0.3f, 1e-6);
  CHECK_NEAR(report.second_power, -0.2f, 1e-6);
  CHECK_NEAR(report.setpoint_rps, 0.0f, 1e-6);
}

void test_bumper_stops_both_wheels() {
  Drivetrain drivetrain = make_drivetrain();
  drivetrain.run_period(kForward, readings(0, 0), kPeriod);
  const PeriodReport report =
      drivetrain.run_period(kForward, readings(0, 0, true), kPeriod);
  CHECK(report.stop_reason == StopReason::kBumper);
  CHECK_NEAR(report.first_power, 0.0f, 1e-6);
  CHECK_NEAR(report.second_power, 0.0f, 1e-6);
}

}  // namespace

int main() {
  test_measures_speed_from_counts();
  test_ramps_toward_a_speed_target();
  test_passes_open_loop_power_through();
  test_bumper_stops_both_wheels();
  return report("drivetrain");
}
