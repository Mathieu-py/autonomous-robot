#pragma once

#include <cstdint>

#include "safety.h"
#include "wheel.h"

// What each wheel should do: a speed for its PID loop, or (for motor
// identification) a fixed power that bypasses it.
struct Target {
  bool is_open_loop_power;
  float first;
  float second;

  bool is_stop() const { return first == 0.0f && second == 0.0f; }
};

// The hardware readings for one control period.
struct Readings {
  int32_t first_count;
  int32_t second_count;
  bool is_bumper_pressed;
  float battery_volts;
};

// What one control period measured and decided, for the motors and the logs.
struct PeriodReport {
  float setpoint_rps;  // the first wheel's, compared to its last reading
  int32_t first_count;
  float first_rps;
  float first_power;
  int32_t second_count;
  float second_rps;
  float second_power;
  float battery_volts;  // filtered
  StopReason stop_reason;
  bool should_pi_shut_down;
};

// Both wheels behind the safety stops. main.cpp reads the hardware and
// applies the powers.
class Drivetrain {
 public:
  Drivetrain(const Wheel& first, const Wheel& second)
      : first_(first), second_(second) {}

  // Measure, check the safety stops, then decide each wheel's power.
  PeriodReport run_period(const Target& target, const Readings& readings,
                          float period_seconds);

 private:
  Wheel first_;
  Wheel second_;
  SafetyMonitor safety_;
};
