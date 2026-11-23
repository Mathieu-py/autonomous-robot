#pragma once

// Conditions that stop the motors.

#include "drive/wheel_state.h"

enum class StopReason {
  kNone,
  kBumper,
  kLowBattery,
  kFirstStall,
  kSecondStall
};

const char* stop_reason_name(StopReason reason);

struct SafetyInputs {
  bool is_bumper_pressed;
  float battery_volts;
  WheelState first;
  WheelState second;
  bool is_stop_commanded;  // the command is zero; needed to clear a stop
  float period_seconds;
};

// How long a condition has held without a break.
class HoldTimer {
 public:
  void update(bool is_holding, float period_seconds) {
    seconds_ = is_holding ? seconds_ + period_seconds : 0.0f;
  }
  bool has_held_for(float seconds) const { return seconds_ >= seconds; }

 private:
  float seconds_ = 0.0f;
};

class SafetyMonitor {
 public:
  // Latches a stop until its cause is gone and a stop is commanded.
  StopReason update(const SafetyInputs& inputs);

  bool should_pi_shut_down() const;
  float filtered_battery_volts() const { return filtered_battery_volts_; }

 private:
  void filter_battery(float battery_volts, float period_seconds);
  void update_low_battery();
  bool is_critically_low() const;
  StopReason active_cause(const SafetyInputs& inputs) const;

  StopReason latched_reason_ = StopReason::kNone;
  bool has_battery_reading_ = false;
  float filtered_battery_volts_ = 0.0f;
  bool is_battery_low_ = false;
  HoldTimer critically_low_timer_;
  HoldTimer first_stall_timer_;
  HoldTimer second_stall_timer_;
};
