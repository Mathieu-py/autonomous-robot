#pragma once

#include <cstdint>

#include "drivetrain.h"
#include "phase_sequence.h"
#include "pico/time.h"

// kDrive takes commands from the Pi; the others are bench tests that wait
// for a serial viewer.
enum class Mode { kDrive, kSpeedTest, kSafetyTest, kSysIdTest };

const char* mode_name(Mode mode);

// A bench test: counts down 3 s once a serial viewer connects, then runs its
// sequence and logs a CSV row every control period.
class BenchTest {
 public:
  explicit BenchTest(Mode mode);

  void start_countdown(absolute_time_t now);
  Target target(absolute_time_t now) const;

  // Logs stop changes, and a CSV row while the sequence runs.
  void log(absolute_time_t now, const PeriodReport& report);

  // Readings for checking the wiring, before the test starts or after it ends.
  void print_idle_status(const PeriodReport& report) const;

 private:
  int64_t elapsed_ms(absolute_time_t now) const;  // negative until the start
  void print_csv_header() const;
  void print_csv_row(int64_t t_ms, const PeriodReport& report) const;

  Mode mode_;
  PhaseSequence sequence_;
  absolute_time_t start_time_ = at_the_end_of_time;  // set on connect
  bool has_started_ = false;
  bool has_finished_ = false;
  StopReason last_stop_reason_ = StopReason::kNone;
};
