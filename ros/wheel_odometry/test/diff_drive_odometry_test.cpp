#include "wheel_odometry/diff_drive_odometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using wheel_odometry::DiffDriveOdometry;
using wheel_odometry::DriveGeometry;
using wheel_odometry::velocity_from_wheels;

namespace {

constexpr double kPi = std::numbers::pi;

constexpr DriveGeometry kGeometry{.wheel_diameter_m = 0.068,
                                  .wheel_separation_m = 0.180};

// Wheel angle that rolls a wheel this far.
double angle_for(double metres) { return metres / kGeometry.wheel_radius_m(); }

// Feeds the wheels' travel in many small steps, as /joint_states does.
void drive(DiffDriveOdometry& odometry, double left_m, double right_m,
           int steps = 100) {
  for (int i = 1; i <= steps; ++i) {
    odometry.update(angle_for(left_m * i / steps),
                    angle_for(right_m * i / steps));
  }
}

}  // namespace

TEST(DiffDriveOdometry, StartsAtZeroWhateverTheWheelAngles) {
  DiffDriveOdometry odometry(kGeometry);
  odometry.update(12.3, -4.5);
  EXPECT_DOUBLE_EQ(odometry.pose().x_m, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose().y_m, 0.0);
  EXPECT_DOUBLE_EQ(odometry.pose().yaw_rad, 0.0);
}

TEST(DiffDriveOdometry, DrivesStraightAhead) {
  DiffDriveOdometry odometry(kGeometry);
  odometry.update(0.0, 0.0);
  drive(odometry, 1.0, 1.0);
  EXPECT_NEAR(odometry.pose().x_m, 1.0, 1e-9);
  EXPECT_NEAR(odometry.pose().y_m, 0.0, 1e-9);
  EXPECT_NEAR(odometry.pose().yaw_rad, 0.0, 1e-9);
}

TEST(DiffDriveOdometry, DrivesStraightAlongItsHeading) {
  DiffDriveOdometry odometry(kGeometry);
  odometry.update(0.0, 0.0);
  // 30 degrees left on the spot, then 1 m straight.
  const double turn_m = kPi / 6.0 * kGeometry.wheel_separation_m / 2;
  odometry.update(angle_for(-turn_m), angle_for(turn_m));
  odometry.update(angle_for(-turn_m + 1.0), angle_for(turn_m + 1.0));
  EXPECT_NEAR(odometry.pose().x_m, std::sqrt(3.0) / 2.0, 1e-9);
  EXPECT_NEAR(odometry.pose().y_m, 0.5, 1e-9);
  EXPECT_NEAR(odometry.pose().yaw_rad, kPi / 6.0, 1e-9);
}

TEST(DiffDriveOdometry, SpinsLeftOnTheSpot) {
  DiffDriveOdometry odometry(kGeometry);
  odometry.update(0.0, 0.0);
  const double quarter_turn_m = kPi / 2.0 * kGeometry.wheel_separation_m / 2;
  drive(odometry, -quarter_turn_m, quarter_turn_m);
  EXPECT_NEAR(odometry.pose().x_m, 0.0, 1e-9);
  EXPECT_NEAR(odometry.pose().y_m, 0.0, 1e-9);
  EXPECT_NEAR(odometry.pose().yaw_rad, kPi / 2.0, 1e-9);
}

TEST(DiffDriveOdometry, QuarterCircleLeftEndsAtTheRightPoint) {
  DiffDriveOdometry odometry(kGeometry);
  odometry.update(0.0, 0.0);
  const double radius = 0.5;
  const double half_track = kGeometry.wheel_separation_m / 2;
  // Large steps: an exact arc doesn't need small ones.
  drive(odometry, kPi / 2 * (radius - half_track),
        kPi / 2 * (radius + half_track), 4);
  EXPECT_NEAR(odometry.pose().x_m, radius, 1e-9);
  EXPECT_NEAR(odometry.pose().y_m, radius, 1e-9);
  EXPECT_NEAR(odometry.pose().yaw_rad, kPi / 2.0, 1e-9);
}

TEST(DiffDriveOdometry, KeepsYawWithinPlusMinusPi) {
  DiffDriveOdometry odometry(kGeometry);
  odometry.update(0.0, 0.0);
  const double three_quarter_turns_m =
      1.5 * kPi * kGeometry.wheel_separation_m / 2;
  drive(odometry, -three_quarter_turns_m, three_quarter_turns_m);
  EXPECT_NEAR(odometry.pose().yaw_rad, -kPi / 2.0, 1e-9);
}

TEST(VelocityFromWheels, SpinningLeftIsPositiveAngular) {
  const auto velocity = velocity_from_wheels(-1.0, 1.0, kGeometry);
  EXPECT_NEAR(velocity.linear_mps, 0.0, 1e-12);
  EXPECT_NEAR(velocity.angular_radps, 2 * 0.034 / 0.180, 1e-12);
}

TEST(VelocityFromWheels, BothForwardIsForward) {
  const auto velocity = velocity_from_wheels(2.0, 2.0, kGeometry);
  EXPECT_NEAR(velocity.linear_mps, 0.068, 1e-12);
  EXPECT_NEAR(velocity.angular_radps, 0.0, 1e-12);
}
