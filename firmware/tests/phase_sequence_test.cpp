#include "bench/phase_sequence.h"

#include <gtest/gtest.h>

namespace {

constexpr Phase kPhases[] = {
    {1.0f, 1.0f, 1000},  // hold
    {0.0f, 2.0f, 2000},  // ramp
};
const PhaseSequence kSequence(kPhases);

TEST(PhaseSequence, NotRunningBeforeTheStart) {
  EXPECT_FALSE(kSequence.is_running(-1));
  EXPECT_NEAR(kSequence.value(-1), 0.0f, 1e-6);
}

TEST(PhaseSequence, HoldsAFlatPhase) {
  EXPECT_TRUE(kSequence.is_running(0));
  EXPECT_EQ(kSequence.phase(999), 0);
  EXPECT_NEAR(kSequence.value(999), 1.0f, 1e-6);
}

TEST(PhaseSequence, RampsWithinAPhase) {
  EXPECT_EQ(kSequence.phase(1000), 1);
  EXPECT_NEAR(kSequence.value(1000), 0.0f, 1e-6);
  EXPECT_NEAR(kSequence.value(2000), 1.0f, 1e-6);
}

TEST(PhaseSequence, StopsAfterTheLastPhase) {
  EXPECT_TRUE(kSequence.is_running(2999));
  EXPECT_FALSE(kSequence.is_running(3000));
  EXPECT_EQ(kSequence.phase(3000), 2);
  EXPECT_NEAR(kSequence.value(3000), 0.0f, 1e-6);
}

}  // namespace
