#include "motor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

namespace {

// 7500 steps at 150 MHz = 20 kHz PWM.
constexpr uint16_t kPwmWrap = 7499;

}  // namespace

Motor::Motor(uint pin_a) : slice_(pwm_gpio_to_slice_num(pin_a)) {
  gpio_set_function(pin_a, GPIO_FUNC_PWM);
  gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);

  pwm_set_clkdiv(slice_, 1.0f);
  pwm_set_wrap(slice_, kPwmWrap);
  pwm_set_both_levels(slice_, 0, 0);
  pwm_set_enabled(slice_, true);
}

void Motor::set_power(float power) const {
  if (std::isnan(power)) {
    power = 0.0f;
  }

  power = std::clamp(power, -kMaxPower, kMaxPower);

  const auto level =
      static_cast<uint16_t>(std::fabs(power) * (kPwmWrap + 1) + 0.5f);

  // Set both at once so A and B are never on together. Zero brakes.
  if (power > 0.0f) {
    pwm_set_both_levels(slice_, level, 0);
  } else {
    pwm_set_both_levels(slice_, 0, level);
  }
}
