#pragma once

#include <cstddef>

#include "pico/time.h"

// Bench-test sequences. Each runs once from start(), timed from that moment;
// is_running() goes false when it finishes.

// PID step test: target forward, stop, then target backward.
class SpeedTest {
 public:
  explicit SpeedTest(float target_rps) : target_rps_(target_rps) {}

  void start(absolute_time_t now);
  bool is_running(absolute_time_t now) const;
  float target_rps(absolute_time_t now) const;  // 0 when not running

 private:
  float target_rps_;
  bool started_ = false;
  absolute_time_t start_time_ = nil_time;
};

// Motor identification: open-loop slow ramps (for k_s, k_v) and power steps
// (for k_a) in each direction, with pauses between.
class SysIdTest {
 public:
  void start(absolute_time_t now);
  bool is_running(absolute_time_t now) const;
  float power(absolute_time_t now) const;  // 0 when not running
  size_t phase(absolute_time_t now) const;

 private:
  bool started_ = false;
  absolute_time_t start_time_ = nil_time;
};
