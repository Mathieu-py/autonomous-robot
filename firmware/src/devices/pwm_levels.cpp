#include "devices/pwm_levels.h"

#include <algorithm>
#include <cmath>

PwmLevels pwm_levels_for(float power, float max_power) {
  if (std::isnan(power)) {
    return {0, 0};
  }
  const float clamped = std::clamp(power, -max_power, max_power);
  const auto level =
      static_cast<uint16_t>(std::fabs(clamped) * (kPwmWrap + 1) + 0.5f);
  return clamped > 0.0f ? PwmLevels{level, 0} : PwmLevels{0, level};
}
