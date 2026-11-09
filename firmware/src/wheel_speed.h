#pragma once

#include <cstdint>

#include "pico/time.h"

// A wheel's speed from its encoder count, updated once per control period.
// Each reading is the average speed over the period since the last update.
class WheelSpeed {
 public:
  WheelSpeed(float ticks_per_rev, int32_t count, absolute_time_t now);

  void update(int32_t count, absolute_time_t now);

  float speed_rps() const { return speed_rps_; }  // revolutions per second
  int32_t count() const { return last_count_; }
  float last_period_seconds() const { return last_period_seconds_; }

 private:
  float ticks_per_rev_;
  int32_t last_count_;
  absolute_time_t last_time_;
  float speed_rps_ = 0.0f;
  float last_period_seconds_ = 0.0f;
};
