#include "drive/safety.h"

#include <cmath>

namespace {

// 3S LiPo, per cell: stop at 3.5 V, Pi off at 3.4 V, re-arm above 3.67 V.
constexpr float kLowBatteryVolts = 10.5f;
constexpr float kPiShutdownVolts = 10.2f;
constexpr float kRecoveredBatteryVolts = 11.0f;
// Below this there's no battery (Pi on its own supply): don't shut it down.
constexpr float kNoBatteryVolts = 6.0f;
// Plugging a battery in or out crosses the shutdown band for ~1 s.
constexpr float kPiShutdownSeconds = 3.0f;

// Averages out the dip when the motors start.
constexpr float kBatteryFilterSeconds = 1.0f;

constexpr float kStallPower = 0.5f;
constexpr float kStallSpeedRps = 0.05f;
constexpr float kStallSeconds = 0.5f;

bool is_pushed_but_still(const WheelState& wheel) {
  return std::fabs(wheel.power) >= kStallPower &&
         std::fabs(wheel.speed_rps) < kStallSpeedRps;
}

}  // namespace

const char* stop_reason_name(StopReason reason) {
  switch (reason) {
    case StopReason::kNone:
      return "none";
    case StopReason::kBumper:
      return "bumper";
    case StopReason::kLowBattery:
      return "low battery";
    case StopReason::kFirstStall:
      return "first motor stalled";
    case StopReason::kSecondStall:
      return "second motor stalled";
  }
  return "unknown";
}

StopReason SafetyMonitor::update(const SafetyInputs& inputs) {
  filter_battery(inputs.battery_volts, inputs.period_seconds);
  update_low_battery();
  critically_low_timer_.update(is_critically_low(), inputs.period_seconds);
  first_stall_timer_.update(is_pushed_but_still(inputs.first),
                            inputs.period_seconds);
  second_stall_timer_.update(is_pushed_but_still(inputs.second),
                             inputs.period_seconds);

  const StopReason cause = active_cause(inputs);
  if (cause != StopReason::kNone) {
    latched_reason_ = cause;
  } else if (inputs.is_stop_commanded) {
    latched_reason_ = StopReason::kNone;
  }
  return latched_reason_;
}

bool SafetyMonitor::should_pi_shut_down() const {
  return critically_low_timer_.has_held_for(kPiShutdownSeconds);
}

void SafetyMonitor::filter_battery(float battery_volts, float period_seconds) {
  if (!has_battery_reading_) {
    has_battery_reading_ = true;
    filtered_battery_volts_ = battery_volts;
  } else {
    const float weight =
        period_seconds / (kBatteryFilterSeconds + period_seconds);
    filtered_battery_volts_ +=
        (battery_volts - filtered_battery_volts_) * weight;
  }
}

void SafetyMonitor::update_low_battery() {
  if (filtered_battery_volts_ < kLowBatteryVolts) {
    is_battery_low_ = true;
  } else if (filtered_battery_volts_ > kRecoveredBatteryVolts) {
    is_battery_low_ = false;
  }
}

bool SafetyMonitor::is_critically_low() const {
  return filtered_battery_volts_ > kNoBatteryVolts &&
         filtered_battery_volts_ < kPiShutdownVolts;
}

StopReason SafetyMonitor::active_cause(const SafetyInputs& inputs) const {
  if (inputs.is_bumper_pressed) {
    return StopReason::kBumper;
  }
  if (is_battery_low_) {
    return StopReason::kLowBattery;
  }
  if (first_stall_timer_.has_held_for(kStallSeconds)) {
    return StopReason::kFirstStall;
  }
  if (second_stall_timer_.has_held_for(kStallSeconds)) {
    return StopReason::kSecondStall;
  }
  return StopReason::kNone;
}
