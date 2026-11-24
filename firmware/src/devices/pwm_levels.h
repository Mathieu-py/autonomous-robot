#pragma once

#include <cstdint>

// 7500 steps at 150 MHz = 20 kHz PWM.
constexpr uint16_t kPwmWrap = 7499;

// One motor's two MDD3A inputs: A drives forward, B backward, both low brakes.
struct PwmLevels {
  uint16_t a;
  uint16_t b;

  friend bool operator==(const PwmLevels&, const PwmLevels&) = default;
};

// Clamped to ±max_power; zero or NaN brakes.
PwmLevels pwm_levels_for(float power, float max_power);
