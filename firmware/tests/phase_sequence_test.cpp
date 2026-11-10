#include "phase_sequence.h"

#include "check.h"

namespace {

constexpr Phase kPhases[] = {
    {1.0f, 1.0f, 1000},  // hold
    {0.0f, 2.0f, 2000},  // ramp
};
const PhaseSequence kSequence(kPhases);

void test_not_running_before_the_start() {
  CHECK(!kSequence.is_running(-1));
  CHECK_NEAR(kSequence.value(-1), 0.0f, 1e-6);
}

void test_holds_a_flat_phase() {
  CHECK(kSequence.is_running(0));
  CHECK(kSequence.phase(999) == 0);
  CHECK_NEAR(kSequence.value(999), 1.0f, 1e-6);
}

void test_ramps_within_a_phase() {
  CHECK(kSequence.phase(1000) == 1);
  CHECK_NEAR(kSequence.value(1000), 0.0f, 1e-6);
  CHECK_NEAR(kSequence.value(2000), 1.0f, 1e-6);
}

void test_stops_after_the_last_phase() {
  CHECK(kSequence.is_running(2999));
  CHECK(!kSequence.is_running(3000));
  CHECK(kSequence.phase(3000) == 2);
  CHECK_NEAR(kSequence.value(3000), 0.0f, 1e-6);
}

}  // namespace

int main() {
  test_not_running_before_the_start();
  test_holds_a_flat_phase();
  test_ramps_within_a_phase();
  test_stops_after_the_last_phase();
  return report("phase_sequence");
}
