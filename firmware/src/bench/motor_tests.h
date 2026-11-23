#pragma once

#include <cstdint>

#include "bench/mode.h"
#include "bench/phase_sequence.h"
#include "drive/drivetrain.h"
#include "pico/time.h"

// The speed, safety and identification tests: each counts down once a serial
// viewer connects, drives the wheels through its phases and logs CSV.
class MotorTest {
 public:
  explicit MotorTest(Mode mode);

  void start_countdown(absolute_time_t now);
  Target target(absolute_time_t now) const;

  // Stop changes, plus a CSV row while running.
  void log(absolute_time_t now, const PeriodReport& report);

  // Readings for checking the wiring, before the test starts or after it ends.
  void print_idle_status(const PeriodReport& report) const;

 private:
  int64_t elapsed_ms(absolute_time_t now) const;  // negative until the start
  void print_csv_row(int64_t t_ms, const PeriodReport& report) const;

  Mode mode_;
  PhaseSequence sequence_;
  absolute_time_t start_time_ = at_the_end_of_time;  // set on connect
  bool has_started_ = false;
  bool has_finished_ = false;
  StopReason last_stop_reason_ = StopReason::kNone;
};
