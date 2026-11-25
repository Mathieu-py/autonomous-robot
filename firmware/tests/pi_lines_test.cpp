#include "pi_lines.h"

#include <string_view>

#include "check.h"
#include "pico_line_examples.hpp"

namespace {

void test_status_line_matches_the_bridges_example() {
  const PeriodReport report = {
      .setpoint_rps = 0.0f,
      .first = {.count = 100, .speed_rps = 0.5f, .power = 0.0f},
      .second = {.count = -200, .speed_rps = -0.25f, .power = 0.0f},
      .filtered_battery_volts = 11.93f,
      .stop_reason = StopReason::kNone,
      .should_pi_shut_down = false};
  CHECK(std::string_view(format_status_line(report).data()) ==
        kExampleStatusLine);
}

void test_imu_line_matches_the_bridges_example() {
  const ImuReading reading = {.accel_mps2 = {0.43f, -0.13f, 9.27f},
                              .gyro_radps = {-0.0295f, 0.0001f, 0.05236f}};
  CHECK(std::string_view(format_imu_line(reading).data()) == kExampleImuLine);
}

}  // namespace

int main() {
  test_status_line_matches_the_bridges_example();
  test_imu_line_matches_the_bridges_example();
  return report("pi_lines");
}
