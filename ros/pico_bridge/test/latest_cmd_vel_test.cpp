#include "pico_bridge/latest_cmd_vel.hpp"

#include <gtest/gtest.h>

#include <chrono>

using namespace std::chrono_literals;
using pico_bridge::LatestCmdVel;

namespace {

const LatestCmdVel::TimePoint kStart{100s};

TEST(LatestCmdVel, IsZeroBeforeTheFirstCommand) {
  const LatestCmdVel latest(1s);
  EXPECT_EQ(latest.current(kStart).linear_mps, 0.0);
}

TEST(LatestCmdVel, ZeroesAStaleCommand) {
  LatestCmdVel latest(1s);
  latest.record({0.1, 0.5}, kStart);
  EXPECT_EQ(latest.current(kStart + 1s).linear_mps, 0.1);
  EXPECT_EQ(latest.current(kStart + 1s).angular_radps, 0.5);
  EXPECT_EQ(latest.current(kStart + 1001ms).linear_mps, 0.0);
  EXPECT_EQ(latest.current(kStart + 1001ms).angular_radps, 0.0);
}

}  // namespace
