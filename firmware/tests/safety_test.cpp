#include "safety.h"

#include "check.h"

namespace {

constexpr float kPeriod = 0.02f;

SafetyInputs running_normally() {
  return {.is_bumper_pressed = false,
          .battery_volts = 12.0f,
          .first_power = 0.35f,
          .first_speed_rps = 1.0f,
          .second_power = 0.35f,
          .second_speed_rps = 1.0f,
          .is_stop_commanded = false,
          .period_seconds = kPeriod};
}

// Feeds the same inputs for the given time and returns the last result.
StopReason run_for(SafetyMonitor& safety, SafetyInputs inputs, float seconds) {
  StopReason reason = StopReason::kNone;
  for (float t = 0.0f; t < seconds; t += kPeriod) {
    reason = safety.update(inputs);
  }
  return reason;
}

void test_normal_running_does_not_stop() {
  SafetyMonitor safety;
  CHECK(run_for(safety, running_normally(), 5.0f) == StopReason::kNone);
}

void test_bumper_stops_and_stays_stopped_until_a_stop_is_commanded() {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_bumper_pressed = true;
  CHECK(safety.update(inputs) == StopReason::kBumper);

  inputs.is_bumper_pressed = false;
  CHECK(run_for(safety, inputs, 1.0f) == StopReason::kBumper);

  inputs.is_stop_commanded = true;
  CHECK(safety.update(inputs) == StopReason::kNone);
}

void test_bumper_held_down_blocks_clearing() {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_bumper_pressed = true;
  inputs.is_stop_commanded = true;
  CHECK(run_for(safety, inputs, 1.0f) == StopReason::kBumper);
}

void test_short_battery_dip_does_not_stop() {
  SafetyMonitor safety;
  run_for(safety, running_normally(), 2.0f);
  SafetyInputs dip = running_normally();
  dip.battery_volts = 9.0f;
  CHECK(run_for(safety, dip, 0.1f) == StopReason::kNone);
}

void test_sustained_low_battery_stops_and_warns_the_pi() {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.battery_volts = 10.3f;
  CHECK(run_for(safety, inputs, 5.0f) == StopReason::kLowBattery);
  CHECK(!safety.should_pi_shut_down());

  inputs.battery_volts = 10.0f;
  run_for(safety, inputs, 5.0f);
  CHECK(safety.should_pi_shut_down());
}

void test_low_battery_only_clears_once_recharged() {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.battery_volts = 10.3f;
  run_for(safety, inputs, 5.0f);

  inputs.is_stop_commanded = true;
  inputs.battery_volts = 10.9f;  // recovers a little once the load is off
  CHECK(run_for(safety, inputs, 5.0f) == StopReason::kLowBattery);

  inputs.battery_volts = 12.0f;
  CHECK(run_for(safety, inputs, 5.0f) == StopReason::kNone);
}

void test_stall_stops_only_after_half_a_second() {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.second_power = 0.6f;
  inputs.second_speed_rps = 0.0f;
  CHECK(run_for(safety, inputs, 0.4f) == StopReason::kNone);
  CHECK(run_for(safety, inputs, 0.2f) == StopReason::kSecondStall);
}

void test_wheel_that_starts_moving_resets_the_stall_timer() {
  SafetyMonitor safety;
  SafetyInputs stalled = running_normally();
  stalled.first_power = 0.6f;
  stalled.first_speed_rps = 0.0f;
  run_for(safety, stalled, 0.4f);
  safety.update(running_normally());
  CHECK(run_for(safety, stalled, 0.4f) == StopReason::kNone);
}

}  // namespace

int main() {
  test_normal_running_does_not_stop();
  test_bumper_stops_and_stays_stopped_until_a_stop_is_commanded();
  test_bumper_held_down_blocks_clearing();
  test_short_battery_dip_does_not_stop();
  test_sustained_low_battery_stops_and_warns_the_pi();
  test_low_battery_only_clears_once_recharged();
  test_stall_stops_only_after_half_a_second();
  test_wheel_that_starts_moving_resets_the_stall_timer();
  return report("safety");
}
