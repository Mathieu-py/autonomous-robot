#include "drive/safety.h"

#include <gtest/gtest.h>

#include <cmath>
#include <initializer_list>

#include "stop_reason_printer.h"

namespace {

constexpr float kPeriod = 0.02f;

SafetyInputs running_normally() {
  return {.is_bumper_pressed = false,
          .battery_volts = 12.0f,
          .is_first_stalling = false,
          .is_second_stalling = false,
          .is_stop_commanded = false,
          .period_seconds = kPeriod};
}

// Feeds the same inputs for the given time and returns the last result.
StopReason run_for(SafetyMonitor& safety, SafetyInputs inputs, float seconds) {
  const long periods = std::lround(seconds / kPeriod);
  StopReason reason = StopReason::kNone;
  for (long i = 0; i < periods; ++i) {
    reason = safety.update(inputs);
  }
  return reason;
}

TEST(SafetyMonitor, NormalRunningDoesNotStop) {
  SafetyMonitor safety;
  EXPECT_EQ(run_for(safety, running_normally(), 5.0f), StopReason::kNone);
}

TEST(SafetyMonitor, BumperStopsAndStaysStoppedUntilAStopIsCommanded) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_bumper_pressed = true;
  EXPECT_EQ(safety.update(inputs), StopReason::kBumper);

  inputs.is_bumper_pressed = false;
  EXPECT_EQ(run_for(safety, inputs, 1.0f), StopReason::kBumper);

  inputs.is_stop_commanded = true;
  EXPECT_EQ(safety.update(inputs), StopReason::kNone);
}

TEST(SafetyMonitor, ReportsTheCauseStillHoldingTheStop) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_bumper_pressed = true;
  safety.update(inputs);

  inputs.is_bumper_pressed = false;
  inputs.battery_volts = 9.0f;
  EXPECT_EQ(run_for(safety, inputs, 5.0f), StopReason::kLowBattery);
}

TEST(SafetyMonitor, BumperHeldDownBlocksClearing) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_bumper_pressed = true;
  inputs.is_stop_commanded = true;
  EXPECT_EQ(run_for(safety, inputs, 1.0f), StopReason::kBumper);
}

TEST(SafetyMonitor, ShortBatteryDipDoesNotStop) {
  SafetyMonitor safety;
  run_for(safety, running_normally(), 2.0f);
  SafetyInputs dip = running_normally();
  dip.battery_volts = 9.0f;
  EXPECT_EQ(run_for(safety, dip, 0.1f), StopReason::kNone);
}

TEST(SafetyMonitor, SustainedLowBatteryStopsAndWarnsThePi) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.battery_volts = 10.3f;
  EXPECT_EQ(run_for(safety, inputs, 5.0f), StopReason::kLowBattery);
  EXPECT_FALSE(safety.should_pi_shut_down());

  inputs.battery_volts = 10.0f;
  run_for(safety, inputs, 5.0f);
  EXPECT_TRUE(safety.should_pi_shut_down());
}

TEST(SafetyMonitor, NoBatteryDoesNotAskThePiToShutDown) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.battery_volts = 0.0f;
  EXPECT_EQ(run_for(safety, inputs, 5.0f), StopReason::kLowBattery);
  EXPECT_FALSE(safety.should_pi_shut_down());
}

TEST(SafetyMonitor, PluggingTheBatteryInOrOutDoesNotAskThePiToShutDown) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  bool was_asked = false;
  for (float volts : {0.0f, 12.0f, 0.0f}) {
    inputs.battery_volts = volts;
    for (float t = 0.0f; t < 5.0f; t += kPeriod) {
      safety.update(inputs);
      was_asked = was_asked || safety.should_pi_shut_down();
    }
  }
  EXPECT_FALSE(was_asked);
}

TEST(SafetyMonitor, LowBatteryOnlyClearsOnceRecharged) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.battery_volts = 10.3f;
  run_for(safety, inputs, 5.0f);

  inputs.is_stop_commanded = true;
  inputs.battery_volts = 10.9f;  // recovers a little once the load is off
  EXPECT_EQ(run_for(safety, inputs, 5.0f), StopReason::kLowBattery);

  inputs.battery_volts = 12.0f;
  EXPECT_EQ(run_for(safety, inputs, 5.0f), StopReason::kNone);
}

TEST(SafetyMonitor, StallStopsOnlyAfterHalfASecond) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_second_stalling = true;
  EXPECT_EQ(run_for(safety, inputs, 0.48f), StopReason::kNone);
  EXPECT_EQ(safety.update(inputs), StopReason::kSecondStall);
}

TEST(SafetyMonitor, FirstWheelStallStops) {
  SafetyMonitor safety;
  SafetyInputs inputs = running_normally();
  inputs.is_first_stalling = true;
  EXPECT_EQ(run_for(safety, inputs, 0.5f), StopReason::kFirstStall);
}

TEST(SafetyMonitor, WheelThatRecoversResetsTheStallTimer) {
  SafetyMonitor safety;
  SafetyInputs stalled = running_normally();
  stalled.is_first_stalling = true;
  run_for(safety, stalled, 0.4f);
  safety.update(running_normally());
  EXPECT_EQ(run_for(safety, stalled, 0.4f), StopReason::kNone);
}

}  // namespace
