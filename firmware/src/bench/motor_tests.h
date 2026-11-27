#pragma once

#include <cstdint>

#include "bench/mode.h"
#include "bench/phase_sequence.h"
#include "drive/drivetrain.h"
#include "periodic_timer.h"
#include "pico/time.h"

// The speed, safety and identification tests: each counts down once a serial
// viewer connects, drives the wheels through its phases and logs CSV.
class MotorTest {
 public:
  MotorTest(Mode mode, absolute_time_t now);

  void start_countdown(absolute_time_t now);

  // For good: the bridge is running, or the viewer wants the wheels stopped.
  void stop_on_input();

  Target target(absolute_time_t now) const;

  // Stop changes, then a CSV row while running, or wiring readings when idle.
  void log(absolute_time_t now, const PeriodReport& report);

 private:
  int64_t elapsed_ms(absolute_time_t now) const;  // negative until the start
  void print_stop_change(StopReason stop_reason);
  void print_idle_status(const PeriodReport& report) const;

  Mode mode_;
  PhaseSequence sequence_;
  absolute_time_t start_time_ = at_the_end_of_time;  // set on connect
  bool is_stopped_by_input_ = false;
  bool was_running_ = false;
  PeriodicTimer idle_status_timer_;
  StopReason last_stop_reason_ = StopReason::kNone;
};
