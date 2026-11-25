#include "periodic_timer.h"

#include <gtest/gtest.h>

namespace {

constexpr uint32_t kPeriodMs = 20;
constexpr float kNoTick = -1.0f;

TEST(PeriodicTimer, TicksOnceEachPeriod) {
  PeriodicTimer timer(0, kPeriodMs);
  EXPECT_FALSE(timer.tick(19'999));
  EXPECT_NEAR(timer.tick(20'000).value_or(kNoTick), 0.020, 1e-6);
  EXPECT_FALSE(timer.tick(20'001));
  EXPECT_NEAR(timer.tick(40'000).value_or(kNoTick), 0.020, 1e-6);
}

TEST(PeriodicTimer, ALateTickKeepsTheSchedule) {
  PeriodicTimer timer(0, kPeriodMs);
  EXPECT_NEAR(timer.tick(25'000).value_or(kNoTick), 0.025, 1e-6);
  // Still due at 40 ms, so it reports the shorter gap.
  EXPECT_FALSE(timer.tick(39'999));
  EXPECT_NEAR(timer.tick(40'000).value_or(kNoTick), 0.015, 1e-6);
}

TEST(PeriodicTimer, SkipsMissedTicksInsteadOfBunching) {
  PeriodicTimer timer(0, kPeriodMs);
  // 20, 40 and 60 ms were missed: one tick, then a full period's wait.
  EXPECT_NEAR(timer.tick(75'000).value_or(kNoTick), 0.075, 1e-6);
  EXPECT_FALSE(timer.tick(94'999));
  EXPECT_NEAR(timer.tick(95'000).value_or(kNoTick), 0.020, 1e-6);
}

TEST(PeriodicTimer, KeepsMicrosecondsLongAfterBoot) {
  constexpr uint64_t kTenDaysUs = 864'000'000'000;
  PeriodicTimer timer(kTenDaysUs, kPeriodMs);
  EXPECT_FALSE(timer.tick(kTenDaysUs + 19'999));
  EXPECT_NEAR(timer.tick(kTenDaysUs + 20'000).value_or(kNoTick), 0.020, 1e-6);
}

}  // namespace
