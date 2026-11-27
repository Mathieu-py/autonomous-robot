#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <optional>

// Pure EKF for a two-wheel robot: fuses wheel odometry with a gyro whose zero
// offset drifts. Learns the offset only while not turning (the wheels miss
// some rotation at each start and stop), and skips the wheels while they slip.
// No ROS.
namespace robot_ekf {

// How fast each quantity can change between readings, as a spread per
// square root of a second.
struct ProcessNoise {
  double accel_mps2;
  double yaw_accel_radps2;
  double gyro_bias_drift_radps;
};

// The tuning from the square tests (docs/milestone-2.md).
inline constexpr ProcessNoise kProcessNoise{
    .accel_mps2 = 1.0, .yaw_accel_radps2 = 5.0, .gyro_bias_drift_radps = 3e-4};
// The bridge removes the gyro's start-up offset; this much may be left.
inline constexpr double kInitialGyroBiasSpreadRadps = 0.002;

struct WheelReading {
  double speed_mps;
  double turn_radps;
  double speed_variance;
  double turn_variance;
};

struct GyroReading {
  double turn_radps;
  double variance;
};

class Ekf {
 public:
  enum Index { kX, kY, kYaw, kSpeed, kTurn, kGyroBias, kSize };
  using Vector = Eigen::Matrix<double, kSize, 1>;
  using Matrix = Eigen::Matrix<double, kSize, kSize>;

  Ekf(const ProcessNoise& noise, double initial_gyro_bias_spread_radps)
      : noise_(noise),
        initial_gyro_bias_variance_(squared(initial_gyro_bias_spread_radps)) {
    restart();
  }

  // At rest at the origin, with the gyro offset unknown again.
  void restart() {
    state_.setZero();
    covariance_.setZero();
    time_s_.reset();
    last_gyro_.reset();
    forget_motion_and_gyro_offset();
  }

  // Moves the estimate on to t_s; an earlier t_s does nothing. After a gap,
  // usually the bridge restarting and re-measuring the gyro offset, the
  // motion and offset start over.
  void predict_to(double t_s) {
    if (!time_s_) {
      time_s_ = t_s;
      return;
    }
    const double dt = t_s - *time_s_;
    if (dt <= 0.0) {
      return;
    }
    time_s_ = t_s;
    if (dt > kMaxPredictS) {
      forget_motion_and_gyro_offset();
      return;
    }

    const double yaw = state_(kYaw);
    const double speed = state_(kSpeed);
    Matrix jacobian = Matrix::Identity();
    jacobian(kX, kYaw) = -speed * std::sin(yaw) * dt;
    jacobian(kX, kSpeed) = std::cos(yaw) * dt;
    jacobian(kY, kYaw) = speed * std::cos(yaw) * dt;
    jacobian(kY, kSpeed) = std::sin(yaw) * dt;
    jacobian(kYaw, kTurn) = dt;

    state_(kX) += speed * std::cos(yaw) * dt;
    state_(kY) += speed * std::sin(yaw) * dt;
    state_(kYaw) = wrapped(yaw + state_(kTurn) * dt);

    Matrix process = Matrix::Zero();
    process(kSpeed, kSpeed) = squared(noise_.accel_mps2) * dt;
    process(kTurn, kTurn) = squared(noise_.yaw_accel_radps2) * dt;
    process(kGyroBias, kGyroBias) = squared(noise_.gyro_bias_drift_radps) * dt;
    covariance_ = jacobian * covariance_ * jacobian.transpose() + process;
  }

  // Returns false when the turn rate was skipped as wheel slip.
  bool update_wheels(const WheelReading& wheels) {
    update(row(kSpeed), wheels.speed_mps, wheels.speed_variance);
    is_turning_ = std::abs(wheels.turn_radps) > kTurningRadps;
    if (has_fresh_gyro()) {
      slip_readings_ = disagrees_with_gyro(wheels) ? slip_readings_ + 1 : 0;
      if (slip_readings_ >= kSlipReadings) {
        return false;
      }
    }
    update(row(kTurn), wheels.turn_radps, wheels.turn_variance,
           is_offset_learnable());
    return true;
  }

  // The gyro reads the turn rate plus its zero offset.
  void update_gyro(const GyroReading& gyro) {
    update(row(kTurn) + row(kGyroBias), gyro.turn_radps, gyro.variance,
           is_offset_learnable());
    last_gyro_ = LastGyro{*time_s_, gyro};
  }

  const Vector& state() const { return state_; }
  const Matrix& covariance() const { return covariance_; }

 private:
  using Row = Eigen::Matrix<double, 1, kSize>;

  struct LastGyro {
    double time_s;
    GyroReading reading;
  };

  // Wheel turn rates this many spreads from the gyro's, for this many readings
  // in a row, are slip: starting and stopping disagree for up to 5.
  static constexpr double kSlipGateSpreads = 4.0;
  static constexpr int kSlipReadings = 8;
  static constexpr double kGyroMaxAgeS = 0.03;
  static constexpr double kMaxPredictS = 1.0;
  // Well above the wheels' turn-rate noise (0.018 rad/s) while driving
  // straight.
  static constexpr double kTurningRadps = 0.05;

  static Row row(Index index) { return Row::Unit(index); }
  static double squared(double value) { return value * value; }
  static double wrapped(double angle_rad) {
    return std::remainder(angle_rad, 2.0 * std::numbers::pi);
  }

  // Keeps the pose; the speed, turn rate and gyro offset start over.
  void forget_motion_and_gyro_offset() {
    state_.segment<3>(kSpeed).setZero();
    covariance_.middleRows<3>(kSpeed).setZero();
    covariance_.middleCols<3>(kSpeed).setZero();
    covariance_.diagonal().segment<3>(kSpeed) << 1.0, 1.0,
        initial_gyro_bias_variance_;
    slip_readings_ = 0;
    is_turning_ = false;
  }

  bool has_fresh_gyro() const {
    return last_gyro_ && *time_s_ - last_gyro_->time_s <= kGyroMaxAgeS;
  }

  bool disagrees_with_gyro(const WheelReading& wheels) const {
    const GyroReading& gyro = last_gyro_->reading;
    const double gyro_turn_radps = gyro.turn_radps - state_(kGyroBias);
    const double spread = std::sqrt(wheels.turn_variance + gyro.variance +
                                    covariance_(kGyroBias, kGyroBias));
    return std::abs(wheels.turn_radps - gyro_turn_radps) >
           kSlipGateSpreads * spread;
  }

  // Only straight or still, with the wheels and gyro agreeing, is any gyro
  // reading its offset.
  bool is_offset_learnable() const {
    return slip_readings_ == 0 && !is_turning_;
  }

  // One scalar reading. A held offset counts as known, so a disagreement moves
  // the turn rate rather than the heading's past. Joseph form keeps the
  // covariance right for either gain.
  void update(const Row& h, double reading, double variance,
              bool can_change_gyro_bias = true) {
    Matrix weighting = covariance_;
    if (!can_change_gyro_bias) {
      weighting.row(kGyroBias).setZero();
      weighting.col(kGyroBias).setZero();
    }
    const double innovation = reading - (h * state_)(0);
    const double innovation_variance =
        (h * weighting * h.transpose())(0) + variance;
    const Vector gain = weighting * h.transpose() / innovation_variance;
    state_ += gain * innovation;
    state_(kYaw) = wrapped(state_(kYaw));
    const Matrix keep = Matrix::Identity() - gain * h;
    covariance_ = keep * covariance_ * keep.transpose() +
                  gain * variance * gain.transpose();
  }

  ProcessNoise noise_;
  double initial_gyro_bias_variance_;
  Vector state_;
  Matrix covariance_;
  std::optional<double> time_s_;
  std::optional<LastGyro> last_gyro_;
  int slip_readings_ = 0;
  bool is_turning_ = false;
};

}  // namespace robot_ekf
