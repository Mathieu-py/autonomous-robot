#include "pico_bridge/drive_protocol.hpp"

#include <gtest/gtest.h>

#include <cmath>

using pico_bridge::DriveGeometry;
using pico_bridge::format_command;
using pico_bridge::parse_status_line;
using pico_bridge::wheel_rps_from_twist;

namespace {

constexpr DriveGeometry kGeometry{.wheel_diameter_m = 0.068,
                                  .wheel_separation_m = 0.20};
const double kMetresPerRev = M_PI * 0.068;

}  // namespace

TEST(WheelRpsFromTwist, StraightAheadDrivesBothWheelsEqually) {
  const auto wheels = wheel_rps_from_twist(0.2, 0.0, kGeometry);
  EXPECT_NEAR(wheels.left, 0.2 / kMetresPerRev, 1e-9);
  EXPECT_NEAR(wheels.right, 0.2 / kMetresPerRev, 1e-9);
}

TEST(WheelRpsFromTwist, TurningLeftOnTheSpotSpinsWheelsOpposite) {
  const auto wheels = wheel_rps_from_twist(0.0, 1.0, kGeometry);
  EXPECT_NEAR(wheels.left, -0.1 / kMetresPerRev, 1e-9);
  EXPECT_NEAR(wheels.right, 0.1 / kMetresPerRev, 1e-9);
}

TEST(FormatCommand, MatchesTheFirmwareProtocol) {
  EXPECT_EQ(format_command(0.5, -0.25), "V 0.500 -0.250\n");
}

TEST(ParseStatusLine, ReadsEveryField) {
  const auto status =
      parse_status_line("S,1234,100,-200,0.500,-0.250,11.93,none\r");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->uptime_ms, 1234);
  EXPECT_EQ(status->first_count, 100);
  EXPECT_EQ(status->second_count, -200);
  EXPECT_DOUBLE_EQ(status->first_rps, 0.5);
  EXPECT_DOUBLE_EQ(status->second_rps, -0.25);
  EXPECT_DOUBLE_EQ(status->battery_volts, 11.93);
  EXPECT_EQ(status->stop_reason, "none");
}

TEST(ParseStatusLine, KeepsStopReasonsWithSpaces) {
  const auto status =
      parse_status_line("S,1,0,0,0.000,0.000,10.40,low battery");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->stop_reason, "low battery");
}

TEST(ParseStatusLine, RejectsOtherLines) {
  EXPECT_FALSE(parse_status_line("SHUTDOWN: battery critically low"));
  EXPECT_FALSE(parse_status_line("S,1,2,3"));
  EXPECT_FALSE(parse_status_line("first: 0 (0.00 rev/s)"));
}
