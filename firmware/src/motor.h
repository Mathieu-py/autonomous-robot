#pragma once

#include "pico/types.h"

// One motor on the MDD3A, driven by two PWM inputs on pin_a and pin_a + 1.
class Motor {
 public:
  // 12 V motors on a 12.6 V battery: 12 / 12.6.
  static constexpr float kMaxPower = 0.95f;

  // pin_a must be even, so pin_a and pin_a + 1 share a PWM slice.
  // The motor starts stopped.
  explicit Motor(uint pin_a);

  // power: -1.0 (full backward) to +1.0 (full forward), as a PWM fraction.
  // Clamped to kMaxPower; zero (or NaN) brakes.
  void set_power(float power) const;

 private:
  uint slice_;
};
