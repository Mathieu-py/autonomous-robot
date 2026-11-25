#include "pico_bridge/pico_protocol.hpp"

#include <gtest/gtest.h>

#include <string>

#include "pico_line_examples.hpp"

using pico_bridge::format_command;
using pico_bridge::is_shutdown_request;
using pico_bridge::kNoStopReason;
using pico_bridge::parse_imu_line;
using pico_bridge::parse_status_line;
using pico_bridge::take_line;

TEST(FormatCommand, MatchesTheExample) {
  EXPECT_EQ(format_command(0.5, -0.25),
            std::string(kExampleCommandLine) + "\n");
}

TEST(TakeLine, ReturnsWholeLinesWithoutLineEndings) {
  std::string received = "S,1\r\n\r\nI,2\r\n";
  EXPECT_EQ(take_line(received), "S,1");
  EXPECT_EQ(take_line(received), "");
  EXPECT_EQ(take_line(received), "I,2");
  EXPECT_FALSE(take_line(received));
  EXPECT_TRUE(received.empty());
}

TEST(TakeLine, KeepsAPartialLineUntilItsNewline) {
  std::string received = "S,1\r\nI,2";
  EXPECT_EQ(take_line(received), "S,1");
  EXPECT_FALSE(take_line(received));
  received += ",3\r\n";
  EXPECT_EQ(take_line(received), "I,2,3");
}

TEST(ParseStatusLine, ReadsEveryFieldOfTheExample) {
  const auto status = parse_status_line(kExampleStatusLine);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->first_count, 100);
  EXPECT_EQ(status->second_count, -200);
  EXPECT_DOUBLE_EQ(status->first_rps, 0.5);
  EXPECT_DOUBLE_EQ(status->second_rps, -0.25);
  EXPECT_DOUBLE_EQ(status->battery_volts, 11.93);
  EXPECT_EQ(status->stop_reason, kNoStopReason);
}

TEST(ParseStatusLine, KeepsStopReasonsWithSpaces) {
  const auto status = parse_status_line("S,0,0,0.000,0.000,10.40,low battery");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->stop_reason, "low battery");
}

TEST(ParseStatusLine, RejectsMalformedSeparators) {
  EXPECT_FALSE(parse_status_line("S,1;2,3,0.000,0.000,11.90,none"));
  EXPECT_FALSE(parse_status_line("S,2,3,0.5x,0.0,11.90,none"));
}

TEST(ParseStatusLine, RejectsOtherLines) {
  EXPECT_FALSE(parse_status_line("SHUTDOWN: battery critically low"));
  EXPECT_FALSE(parse_status_line("S,2,3"));
  EXPECT_FALSE(parse_status_line("first: 0 (0.00 rev/s)"));
}

TEST(ParseImuLine, ReadsEveryFieldOfTheExample) {
  const auto sample = parse_imu_line(kExampleImuLine);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->accel_mps2.x, 0.43);
  EXPECT_DOUBLE_EQ(sample->accel_mps2.y, -0.13);
  EXPECT_DOUBLE_EQ(sample->accel_mps2.z, 9.27);
  EXPECT_DOUBLE_EQ(sample->gyro_radps.x, -0.0295);
  EXPECT_DOUBLE_EQ(sample->gyro_radps.y, 0.0001);
  EXPECT_DOUBLE_EQ(sample->gyro_radps.z, 0.05236);
}

TEST(ParseImuLine, RejectsOtherLines) {
  EXPECT_FALSE(parse_imu_line("S,0,0,0.000,0.000,11.90,none"));
  EXPECT_FALSE(parse_imu_line("I,0.1,0.2"));
  EXPECT_FALSE(parse_imu_line("I,0.1;0.2,0.3,0.4,0.5,0.6,0.7"));
  EXPECT_FALSE(parse_imu_line("WARNING: the IMU didn't answer on I2C"));
}

TEST(IsShutdownRequest, RecognisesOnlyThePicosShutdownLine) {
  EXPECT_TRUE(is_shutdown_request("SHUTDOWN: battery critically low"));
  EXPECT_FALSE(is_shutdown_request("S,0,0,0.000,0.000,10.10,low battery"));
  EXPECT_FALSE(
      is_shutdown_request("Rebooted by the watchdog: the firmware had hung"));
}
