#include "devices/pwm_levels.h"

#include <gtest/gtest.h>

#include <limits>
#include <ostream>

// Shows the levels when a check fails.
inline void PrintTo(const PwmLevels& levels, std::ostream* os) {
  *os << "{a " << levels.a << ", b " << levels.b << "}";
}

namespace {

constexpr float kMaxPower = 0.95f;
constexpr PwmLevels kBrake = {0, 0};

TEST(PwmLevelsFor, ForwardDrivesAOnly) {
  EXPECT_EQ(pwm_levels_for(0.5f, kMaxPower), (PwmLevels{3750, 0}));
}

TEST(PwmLevelsFor, BackwardDrivesBOnly) {
  EXPECT_EQ(pwm_levels_for(-0.5f, kMaxPower), (PwmLevels{0, 3750}));
}

TEST(PwmLevelsFor, ClampsToMaxPower) {
  // 0.95 of the 7500 steps.
  EXPECT_EQ(pwm_levels_for(2.0f, kMaxPower), (PwmLevels{7125, 0}));
  EXPECT_EQ(pwm_levels_for(-2.0f, kMaxPower), (PwmLevels{0, 7125}));
}

TEST(PwmLevelsFor, ZeroAndNanBrake) {
  EXPECT_EQ(pwm_levels_for(0.0f, kMaxPower), kBrake);
  EXPECT_EQ(pwm_levels_for(-0.0f, kMaxPower), kBrake);
  EXPECT_EQ(pwm_levels_for(std::numeric_limits<float>::quiet_NaN(), kMaxPower),
            kBrake);
}

}  // namespace
