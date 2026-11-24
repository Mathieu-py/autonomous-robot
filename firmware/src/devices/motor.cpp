#include "devices/motor.h"

#include "devices/pwm_levels.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

Motor::Motor(uint pin_a) : slice_(pwm_gpio_to_slice_num(pin_a)) {
  gpio_set_function(pin_a, GPIO_FUNC_PWM);
  gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);

  pwm_set_clkdiv(slice_, 1.0f);
  pwm_set_wrap(slice_, kPwmWrap);
  pwm_set_both_levels(slice_, 0, 0);
  pwm_set_enabled(slice_, true);
}

void Motor::set_power(float power) const {
  const PwmLevels levels = pwm_levels_for(power, kMaxPower);
  // Both at once, so A and B are never on together.
  pwm_set_both_levels(slice_, levels.a, levels.b);
}
