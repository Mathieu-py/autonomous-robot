#include "drive/wheel.h"

#include <gtest/gtest.h>

namespace {

constexpr float kPeriod = 0.02f;
constexpr Wheel::Config kConfig = {
    .ticks_per_rev = 1000.0f,
    .max_accel_rps2 = 5.0f,
    .max_power = 0.95f,
    .gains = {.k_s = 0.012f,
              .k_v = 0.345f,
              .k_a = 0.0174f,
              .k_p = 0.277f,
              .k_i = 5.48f},
};

// A wheel that turned at speed_rps over the last period while given power.
Wheel wheel_at(float speed_rps, float power) {
  Wheel wheel(kConfig, 0);
  const float ticks = speed_rps * kConfig.ticks_per_rev * kPeriod;
  wheel.measure(static_cast<int32_t>(ticks), kPeriod);
  wheel.set_power(power);
  return wheel;
}

TEST(Wheel, HeldStillUnderPowerIsStalling) {
  EXPECT_TRUE(wheel_at(0.0f, 0.5f).is_stalling());
  EXPECT_TRUE(wheel_at(0.0f, -0.5f).is_stalling());
}

TEST(Wheel, HeldButCreepingUnderFullPowerIsStalling) {
  // Safety run 01: a hand-held wheel still turned 0.4 rev/s at 0.95 power.
  EXPECT_TRUE(wheel_at(0.4f, 0.95f).is_stalling());
}

TEST(Wheel, TurningAgainstItsPowerIsStalling) {
  // A backwards encoder reads the speed opposite to the power.
  EXPECT_TRUE(wheel_at(-1.0f, 0.3f).is_stalling());
}

TEST(Wheel, PushedHardAgainstItsPowerIsStalling) {
  // Back-EMF adds to the drive: the current is above stall.
  EXPECT_TRUE(wheel_at(2.0f, 0.1f).is_stalling());
}

TEST(Wheel, TrackingItsSpeedIsNotStalling) {
  EXPECT_FALSE(wheel_at(1.0f, 0.36f).is_stalling());
  EXPECT_FALSE(wheel_at(-1.0f, -0.36f).is_stalling());
  // Flat out at the low-battery cutoff.
  EXPECT_FALSE(wheel_at(2.2f, 0.95f).is_stalling());
}

TEST(Wheel, PushedWhileBrakedIsNotStalling) {
  // Zero power already brakes, so a stop can't help.
  EXPECT_FALSE(wheel_at(1.0f, 0.0f).is_stalling());
}

}  // namespace
