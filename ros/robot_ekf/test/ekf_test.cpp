#include "robot_ekf/ekf.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <numbers>

using robot_ekf::Ekf;
using robot_ekf::ProcessNoise;

namespace {

constexpr ProcessNoise kNoise{
    .accel_mps2 = 1.0, .yaw_accel_radps2 = 5.0, .gyro_bias_drift_radps = 3e-4};
// The bridge removes the gyro's start-up offset; this much may be left.
constexpr double kInitialGyroBiasSpreadRadps = 0.002;
constexpr double kPeriodS = 0.02;
constexpr double kSpeedVariance = 2.2e-6;  // as measured on the robot
constexpr double kTurnVariance = 3.1e-4;
constexpr double kGyroVariance = 4.1e-4;
constexpr double kDegree = std::numbers::pi / 180.0;

// One 20 ms reading from each sensor, wheels first as on the robot. Returns
// whether the wheels' turn rate was used.
bool step(Ekf& ekf, double& t_s, double speed_mps, double wheel_turn_radps,
          double gyro_turn_radps) {
  t_s += kPeriodS;
  ekf.predict_to(t_s);
  const bool is_turn_used = ekf.update_wheels(speed_mps, wheel_turn_radps,
                                              kSpeedVariance, kTurnVariance);
  ekf.update_gyro(gyro_turn_radps, kGyroVariance);
  return is_turn_used;
}

void drive(Ekf& ekf, double& t_s, int readings, double speed_mps,
           double wheel_turn_radps, double gyro_turn_radps) {
  for (int i = 0; i < readings; ++i) {
    step(ekf, t_s, speed_mps, wheel_turn_radps, gyro_turn_radps);
  }
}

Ekf settled_ekf(double& t_s) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  drive(ekf, t_s, 1500, 0.0, 0.0, 0.0);  // 30 s still: the offset settles
  return ekf;
}

}  // namespace

TEST(Ekf, DrivesStraightForward) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  drive(ekf, t_s, 500, 0.1, 0.0, 0.0);  // 10 s
  EXPECT_NEAR(ekf.state()(Ekf::kX), 1.0, 0.003);
  EXPECT_NEAR(ekf.state()(Ekf::kY), 0.0, 1e-9);
}

TEST(Ekf, GrowsLessSureOfThePositionWhileDriving) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  drive(ekf, t_s, 500, 0.1, 0.0, 0.0);
  const Ekf::Matrix after_1_m = ekf.covariance();
  drive(ekf, t_s, 500, 0.1, 0.0, 0.0);
  EXPECT_GT(ekf.covariance()(Ekf::kX, Ekf::kX), after_1_m(Ekf::kX, Ekf::kX));
  EXPECT_GT(ekf.covariance()(Ekf::kY, Ekf::kY), after_1_m(Ekf::kY, Ekf::kY));
}

TEST(Ekf, LearnsAGyroOffsetAndKeepsTheHeading) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  // The wheels say straight; the gyro reads an offset of 0.06 deg/s.
  drive(ekf, t_s, 3000, 0.1, 0.0, 0.001);  // 60 s
  EXPECT_NEAR(ekf.state()(Ekf::kGyroBias), 0.001, 0.0002);
  EXPECT_LT(std::abs(ekf.state()(Ekf::kYaw)), 0.2 * kDegree);
}

TEST(Ekf, FollowsAQuarterTurn) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  const double turn_radps = std::numbers::pi / 4.0;
  drive(ekf, t_s, 100, 0.0, turn_radps, turn_radps);  // 2 s
  EXPECT_NEAR(ekf.state()(Ekf::kYaw), std::numbers::pi / 2.0, 1.0 * kDegree);
}

TEST(Ekf, SpinsTwiceWithAWrappedHeadingAndAValidCovariance) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  const double turn_radps = 4.0 * std::numbers::pi / (1250 * kPeriodS);
  drive(ekf, t_s, 1250, 0.0, turn_radps, turn_radps);  // 720 deg
  EXPECT_NEAR(ekf.state()(Ekf::kYaw), 0.0, 1.0 * kDegree);
  const Ekf::Matrix& p = ekf.covariance();
  EXPECT_LT((p - p.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  const Eigen::SelfAdjointEigenSolver<Ekf::Matrix> solver(p);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(Ekf, TurnsByTheGyroWhileTheWheelsSlip) {
  double t_s = 0.0;
  Ekf ekf = settled_ekf(t_s);
  // The gyro sees 0.5 rad/s for 1 s; the slipping wheels see nothing.
  drive(ekf, t_s, 50, 0.0, 0.0, 0.5);
  // About 80% of the 0.5 rad: the first 140 ms count as ordinary
  // disagreement. Only the slip's first gyro reading teaches the offset.
  EXPECT_NEAR(ekf.state()(Ekf::kYaw), 0.41, 0.02);
  EXPECT_NEAR(ekf.state()(Ekf::kGyroBias), 0.0, 0.001);
}

TEST(Ekf, BriefDisagreementIsNotSlip) {
  double t_s = 0.0;
  Ekf ekf = settled_ekf(t_s);
  // Starting a spin, the wheels lead the gyro for up to 5 readings.
  for (int i = 0; i < 7; ++i) {
    EXPECT_TRUE(step(ekf, t_s, 0.0, 0.5, 0.0));
  }
}

TEST(Ekf, TrustsTheWheelsAgainOnceTheyAgree) {
  double t_s = 0.0;
  Ekf ekf = settled_ekf(t_s);
  drive(ekf, t_s, 50, 0.0, 0.0, 0.5);
  // Each wheel reading is checked against the gyro reading before it.
  EXPECT_FALSE(step(ekf, t_s, 0.0, 0.0, 0.0));
  EXPECT_TRUE(step(ekf, t_s, 0.0, 0.0, 0.0));
}

TEST(Ekf, ChecksTheWheelsAgainstTheOffsetCorrectedGyro) {
  Ekf ekf(kNoise, 0.5);
  double t_s = 0.0;
  drive(ekf, t_s, 1500, 0.0, 0.0, 0.2);  // learns a 0.2 rad/s offset
  for (int i = 0; i < 50; ++i) {
    EXPECT_TRUE(step(ekf, t_s, 0.0, 0.5, 0.7));
  }
}

TEST(Ekf, UsesTheWheelsWhenTheGyroStops) {
  double t_s = 0.0;
  Ekf ekf = settled_ekf(t_s);
  drive(ekf, t_s, 50, 0.0, 0.0, 0.5);  // a slip, then the gyro goes quiet
  for (int i = 0; i < 2; ++i) {
    t_s += kPeriodS;
    ekf.predict_to(t_s);
    ekf.update_wheels(0.0, 0.0, kSpeedVariance, kTurnVariance);
  }
  t_s += kPeriodS;
  ekf.predict_to(t_s);
  EXPECT_TRUE(ekf.update_wheels(0.0, 0.0, kSpeedVariance, kTurnVariance));
}

TEST(Ekf, CatchesASlipThroughMissedGyroReadings) {
  double t_s = 0.0;
  Ekf ekf = settled_ekf(t_s);
  bool was_slip_caught = false;
  for (int i = 0; i < 50; ++i) {
    t_s += kPeriodS;
    ekf.predict_to(t_s);
    if (!ekf.update_wheels(0.0, 0.0, kSpeedVariance, kTurnVariance)) {
      was_slip_caught = true;
    }
    if (i % 3 != 2) {  // every third gyro reading lost
      ekf.update_gyro(0.5, kGyroVariance);
    }
  }
  EXPECT_TRUE(was_slip_caught);
}

TEST(Ekf, DoesNotExtrapolateAcrossAGap) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  drive(ekf, t_s, 50, 0.1, 0.0, 0.0);
  const double x_m = ekf.state()(Ekf::kX);
  t_s += 5.0;  // the sensors stopped, or the clock stepped
  ekf.predict_to(t_s);
  EXPECT_EQ(ekf.state()(Ekf::kX), x_m);
}

TEST(Ekf, PredictsNothingForAnEarlierTime) {
  Ekf ekf(kNoise, kInitialGyroBiasSpreadRadps);
  double t_s = 0.0;
  drive(ekf, t_s, 50, 0.1, 0.0, 0.0);
  const double x_m = ekf.state()(Ekf::kX);
  ekf.predict_to(t_s - 0.5);
  EXPECT_EQ(ekf.state()(Ekf::kX), x_m);
}
