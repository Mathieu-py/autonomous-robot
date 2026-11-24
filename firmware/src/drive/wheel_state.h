#pragma once

#include <cstdint>

// One wheel after a control period: where it is, how fast, and its power.
struct WheelState {
  int32_t count;
  float speed_rps;
  float power;  // the last one set; runs until the next control period
};
