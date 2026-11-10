#include "safety.h"

#include <cmath>

namespace {

// 3S LiPo: stop the motors at 3.5 V per cell, ask the Pi to shut down at
// 3.4 V, and only re-arm once the pack reads above 3.67 V (recharged).
constexpr float kLowBatteryVolts = 10.5f;
constexpr float kPiShutdownVolts = 10.2f;
constexpr float kRecoveredBatteryVolts = 11.0f;

// Averages out the dip when the motors start.
constexpr float kBatteryFilterSeconds = 1.0f;

constexpr float kStallPower = 0.5f;
constexpr float kStallSpeedRps = 0.05f;
constexpr float kStallSeconds = 0.5f;

bool is_stalling(float power, float speed_rps) {
  return std::fabs(power) >= kStallPower &&
         std::fabs(speed_rps) < kStallSpeedRps;
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
  update_stall_timers(inputs);

  const StopReason cause = active_cause(inputs);
  if (latched_reason_ == StopReason::kNone) {
    latched_reason_ = cause;
  } else if (cause == StopReason::kNone && inputs.is_stop_commanded) {
    latched_reason_ = StopReason::kNone;
  }
  return latched_reason_;
}

StopReason SafetyMonitor::active_cause(const SafetyInputs& inputs) const {
  if (inputs.is_bumper_pressed) {
    return StopReason::kBumper;
  }
  if (is_battery_low_) {
    return StopReason::kLowBattery;
  }
  if (first_stall_seconds_ >= kStallSeconds) {
    return StopReason::kFirstStall;
  }
  if (second_stall_seconds_ >= kStallSeconds) {
    return StopReason::kSecondStall;
  }
  return StopReason::kNone;
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

  if (filtered_battery_volts_ < kLowBatteryVolts) {
    is_battery_low_ = true;
  } else if (filtered_battery_volts_ > kRecoveredBatteryVolts) {
    is_battery_low_ = false;
  }
  should_pi_shut_down_ = filtered_battery_volts_ < kPiShutdownVolts;
}

void SafetyMonitor::update_stall_timers(const SafetyInputs& inputs) {
  first_stall_seconds_ = is_stalling(inputs.first_power, inputs.first_speed_rps)
                             ? first_stall_seconds_ + inputs.period_seconds
                             : 0.0f;
  second_stall_seconds_ =
      is_stalling(inputs.second_power, inputs.second_speed_rps)
          ? second_stall_seconds_ + inputs.period_seconds
          : 0.0f;
}
