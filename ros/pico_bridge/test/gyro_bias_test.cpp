#include "pico_bridge/gyro_bias.hpp"

#include <gtest/gtest.h>

using pico_bridge::GyroBias;
using pico_bridge::Vector3;
using Progress = GyroBias::Progress;

namespace {

constexpr double kMaxSpreadRadps = 0.035;

}  // namespace

TEST(GyroBias, AveragesAStillWindow) {
  GyroBias bias(4, kMaxSpreadRadps);
  EXPECT_EQ(bias.add({0.010, -0.020, -0.050}), Progress::kMeasuring);
  EXPECT_EQ(bias.add({0.012, -0.022, -0.054}), Progress::kMeasuring);
  EXPECT_EQ(bias.add({0.010, -0.020, -0.050}), Progress::kMeasuring);
  EXPECT_EQ(bias.add({0.012, -0.022, -0.054}), Progress::kReady);
  EXPECT_NEAR(bias.bias().x, 0.011, 1e-12);
  EXPECT_NEAR(bias.bias().y, -0.021, 1e-12);
  EXPECT_NEAR(bias.bias().z, -0.052, 1e-12);
}

TEST(GyroBias, RemovesTheOffset) {
  GyroBias bias(1, kMaxSpreadRadps);
  bias.add({0.01, 0.02, -0.05});
  const Vector3 corrected = bias.remove_from({0.01, 0.02, 0.45});
  EXPECT_NEAR(corrected.x, 0.0, 1e-12);
  EXPECT_NEAR(corrected.y, 0.0, 1e-12);
  EXPECT_NEAR(corrected.z, 0.5, 1e-12);
}

TEST(GyroBias, RestartsWhenTheRobotTurns) {
  GyroBias bias(3, kMaxSpreadRadps);
  bias.add({0.0, 0.0, -0.05});
  EXPECT_EQ(bias.add({0.0, 0.0, 0.10}), Progress::kMovedSoRestarted);
  EXPECT_FALSE(bias.is_ready());
  // The turn doesn't leak into the next window.
  bias.add({0.0, 0.0, -0.05});
  bias.add({0.0, 0.0, -0.05});
  EXPECT_EQ(bias.add({0.0, 0.0, -0.05}), Progress::kReady);
  EXPECT_NEAR(bias.bias().z, -0.05, 1e-12);
}

TEST(GyroBias, KeepsTheFirstOffsetOnceReady) {
  GyroBias bias(1, kMaxSpreadRadps);
  bias.add({0.0, 0.0, -0.05});
  EXPECT_EQ(bias.add({0.0, 0.0, 1.0}), Progress::kReady);
  EXPECT_NEAR(bias.bias().z, -0.05, 1e-12);
}
