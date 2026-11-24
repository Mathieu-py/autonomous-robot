#include "devices/pwm_levels.h"

#include <limits>

#include "check.h"

namespace {

constexpr float kMaxPower = 0.95f;

bool brakes(float power) {
  return pwm_levels_for(power, kMaxPower) == PwmLevels{0, 0};
}

void test_forward_drives_a_only() {
  CHECK(pwm_levels_for(0.5f, kMaxPower) == (PwmLevels{3750, 0}));
}

void test_backward_drives_b_only() {
  CHECK(pwm_levels_for(-0.5f, kMaxPower) == (PwmLevels{0, 3750}));
}

void test_clamps_to_max_power() {
  // 0.95 of the 7500 steps.
  CHECK(pwm_levels_for(2.0f, kMaxPower) == (PwmLevels{7125, 0}));
  CHECK(pwm_levels_for(-2.0f, kMaxPower) == (PwmLevels{0, 7125}));
}

void test_zero_and_nan_brake() {
  CHECK(brakes(0.0f));
  CHECK(brakes(-0.0f));
  CHECK(brakes(std::numeric_limits<float>::quiet_NaN()));
}

}  // namespace

int main() {
  test_forward_drives_a_only();
  test_backward_drives_b_only();
  test_clamps_to_max_power();
  test_zero_and_nan_brake();
  return report("pwm_levels");
}
