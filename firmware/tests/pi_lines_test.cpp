#include "pi_lines.h"

#include <gtest/gtest.h>

#include <string_view>

#include "pico_line_examples.hpp"

namespace {

TEST(FormatStatusLine, MatchesTheBridgesExample) {
  const PeriodReport report = {
      .setpoint_rps = 0.0f,
      .first = {.count = 100, .speed_rps = 0.5f, .power = 0.0f},
      .second = {.count = -200, .speed_rps = -0.25f, .power = 0.0f},
      .filtered_battery_volts = 11.93f,
      .stop_reason = StopReason::kNone,
      .should_pi_shut_down = false};
  EXPECT_EQ(std::string_view(format_status_line(report).data()),
            kExampleStatusLine);
}

TEST(FormatImuLine, MatchesTheBridgesExample) {
  const ImuReading reading = {.accel_mps2 = {0.43f, -0.13f, 9.27f},
                              .gyro_radps = {-0.0295f, 0.0001f, 0.05236f}};
  EXPECT_EQ(std::string_view(format_imu_line(reading).data()), kExampleImuLine);
}

TEST(ShutdownLine, MatchesTheBridgesExample) {
  EXPECT_STREQ(kShutdownLine, kExampleShutdownLine);
}

}  // namespace
