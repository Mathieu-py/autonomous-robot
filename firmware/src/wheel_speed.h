#pragma once

#include <cstdint>

// A wheel's speed from its encoder count, updated once per control period.
// Each reading is the average speed over the period since the last update.
class WheelSpeed {
 public:
  WheelSpeed(float ticks_per_rev, int32_t count)
      : ticks_per_rev_(ticks_per_rev), last_count_(count) {}

  void update(int32_t count, float period_seconds);

  float speed_rps() const { return speed_rps_; }  // revolutions per second
  int32_t count() const { return last_count_; }

 private:
  float ticks_per_rev_;
  int32_t last_count_;
  float speed_rps_ = 0.0f;
};
