#pragma once

// Conditions that stop the motors. Pure logic, so it can be unit-tested off
// the Pico; main.cpp reads the hardware and passes the readings in.

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
  float first_power;  // applied over the period just measured
  float first_speed_rps;
  float second_power;
  float second_speed_rps;
  bool is_stop_commanded;  // the command is zero; needed to clear a stop
  float period_seconds;
};

class SafetyMonitor {
 public:
  // Latches a stop until its cause is gone and a stop is commanded.
  StopReason update(const SafetyInputs& inputs);

  bool should_pi_shut_down() const { return should_pi_shut_down_; }
  float filtered_battery_volts() const { return filtered_battery_volts_; }

 private:
  StopReason active_cause(const SafetyInputs& inputs) const;
  void filter_battery(float battery_volts, float period_seconds);
  void time_stalls(const SafetyInputs& inputs);

  StopReason latched_ = StopReason::kNone;
  bool has_battery_reading_ = false;
  float filtered_battery_volts_ = 0.0f;
  bool is_battery_low_ = false;
  bool should_pi_shut_down_ = false;
  float first_stall_seconds_ = 0.0f;
  float second_stall_seconds_ = 0.0f;
};
