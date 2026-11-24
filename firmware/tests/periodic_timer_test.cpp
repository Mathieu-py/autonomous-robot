#include "periodic_timer.h"

#include "check.h"

namespace {

constexpr uint32_t kPeriodMs = 20;
constexpr float kNoTick = -1.0f;

void test_ticks_once_each_period() {
  PeriodicTimer timer(0, kPeriodMs);
  CHECK(!timer.tick(19'999));
  CHECK_NEAR(timer.tick(20'000).value_or(kNoTick), 0.020, 1e-6);
  CHECK(!timer.tick(20'001));
  CHECK_NEAR(timer.tick(40'000).value_or(kNoTick), 0.020, 1e-6);
}

void test_a_late_tick_keeps_the_schedule() {
  PeriodicTimer timer(0, kPeriodMs);
  CHECK_NEAR(timer.tick(25'000).value_or(kNoTick), 0.025, 1e-6);
  // Still due at 40 ms, so it reports the shorter gap.
  CHECK(!timer.tick(39'999));
  CHECK_NEAR(timer.tick(40'000).value_or(kNoTick), 0.015, 1e-6);
}

void test_skips_missed_ticks_instead_of_bunching() {
  PeriodicTimer timer(0, kPeriodMs);
  // 20, 40 and 60 ms were missed: one tick, then a full period's wait.
  CHECK_NEAR(timer.tick(75'000).value_or(kNoTick), 0.075, 1e-6);
  CHECK(!timer.tick(94'999));
  CHECK_NEAR(timer.tick(95'000).value_or(kNoTick), 0.020, 1e-6);
}

void test_keeps_microseconds_long_after_boot() {
  constexpr uint64_t kTenDaysUs = 864'000'000'000;
  PeriodicTimer timer(kTenDaysUs, kPeriodMs);
  CHECK(!timer.tick(kTenDaysUs + 19'999));
  CHECK_NEAR(timer.tick(kTenDaysUs + 20'000).value_or(kNoTick), 0.020, 1e-6);
}

}  // namespace

int main() {
  test_ticks_once_each_period();
  test_a_late_tick_keeps_the_schedule();
  test_skips_missed_ticks_instead_of_bunching();
  test_keeps_microseconds_long_after_boot();
  return report("periodic_timer");
}
