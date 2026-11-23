#pragma once

#include <cstdint>

#include "drive/safety.h"
#include "drive/wheel.h"

// A speed for each PID loop, or a fixed power (motor identification).
struct Target {
  bool is_open_loop_power;
  float first;
  float second;

  bool is_stop() const { return first == 0.0f && second == 0.0f; }
};

struct Readings {
  int32_t first_count;
  int32_t second_count;
  bool is_bumper_pressed;
  float battery_volts;
};

// One control period's results, for the motors and the logs.
struct PeriodReport {
  float setpoint_rps;  // the first wheel's, compared to its last reading
  WheelState first;
  WheelState second;
  float battery_volts;  // filtered
  StopReason stop_reason;
  bool should_pi_shut_down;
};

// Both wheels behind the safety stops; main.cpp does the hardware I/O.
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
