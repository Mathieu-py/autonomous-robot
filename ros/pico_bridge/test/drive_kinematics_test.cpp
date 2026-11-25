#include "pico_bridge/drive_kinematics.hpp"

#include <gtest/gtest.h>

#include <numbers>

using pico_bridge::DriveGeometry;
using pico_bridge::MotorMapping;
using pico_bridge::wheel_rps_from_twist;

namespace {

constexpr DriveGeometry kGeometry{.wheel_diameter_m = 0.068,
                                  .wheel_separation_m = 0.20};
constexpr double kMetresPerRev = std::numbers::pi * 0.068;

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

TEST(MotorMapping, OurRobotReversesTheRightMotor) {
  const MotorMapping mapping{.is_left_first_motor = true,
                             .is_left_reversed = false,
                             .is_right_reversed = true};
  const auto command = mapping.to_motors({.left = 0.5, .right = 0.5});
  EXPECT_DOUBLE_EQ(command.first, 0.5);
  EXPECT_DOUBLE_EQ(command.second, -0.5);
}

TEST(MotorMapping, SwapsMotorsWhenTheLeftWheelIsSecond) {
  const MotorMapping mapping{.is_left_first_motor = false,
                             .is_left_reversed = false,
                             .is_right_reversed = false};
  const auto command = mapping.to_motors({.left = 0.1, .right = 0.2});
  EXPECT_DOUBLE_EQ(command.first, 0.2);
  EXPECT_DOUBLE_EQ(command.second, 0.1);
}

TEST(MotorMapping, ReadingsMapBackToTheSameWheels) {
  for (const bool is_left_first : {true, false}) {
    for (const bool is_right_reversed : {true, false}) {
      const MotorMapping mapping{.is_left_first_motor = is_left_first,
                                 .is_left_reversed = true,
                                 .is_right_reversed = is_right_reversed};
      const auto wheels = mapping.to_wheels(mapping.to_motors({0.3, -0.7}));
      EXPECT_DOUBLE_EQ(wheels.left, 0.3);
      EXPECT_DOUBLE_EQ(wheels.right, -0.7);
    }
  }
}
