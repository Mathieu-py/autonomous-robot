#pragma once

#include "pico/types.h"

// One motor on the MDD3A, driven by two PWM inputs on pin_a and pin_a + 1.
class Motor {
 public:
  // 12 V motors on a 12.6 V battery.
  static constexpr float kMaxPower = 0.95f;

  // pin_a must be even (A and B share a PWM slice). Starts stopped.
  explicit Motor(uint pin_a);

  // -1 (full back) to +1 (full forward), clamped; zero or NaN brakes.
  void set_power(float power) const;

 private:
  uint slice_;
};
