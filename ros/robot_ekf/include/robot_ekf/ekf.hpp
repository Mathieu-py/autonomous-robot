#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <optional>

// Pure EKF for a two-wheel robot: fuses wheel odometry with a gyro whose zero
// offset drifts, estimates that offset, and skips the wheels' turn rate while
// they slip. No ROS.
namespace robot_ekf {

// How fast each quantity can change between readings, as a spread per
// square root of a second.
struct ProcessNoise {
  double accel_mps2;
  double yaw_accel_radps2;
  double gyro_bias_drift_radps;
};

class Ekf {
 public:
  enum Index { kX, kY, kYaw, kSpeed, kTurn, kGyroBias, kSize };
  using Vector = Eigen::Matrix<double, kSize, 1>;
  using Matrix = Eigen::Matrix<double, kSize, kSize>;

  // Starts at rest at the origin; the gyro offset is unknown by about this.
  Ekf(const ProcessNoise& noise, double initial_gyro_bias_spread_radps)
      : noise_(noise) {
    covariance_.diagonal() << 0.0, 0.0, 0.0, 1.0, 1.0,
        initial_gyro_bias_spread_radps * initial_gyro_bias_spread_radps;
  }

  // Moves the estimate on to t_s; an earlier t_s does nothing. Across a gap
  // the sensors had stopped, so it doesn't extrapolate.
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
  bool update_wheels(double speed_mps, double turn_radps, double speed_variance,
                     double turn_variance) {
    update(row(kSpeed), speed_mps, speed_variance);
    if (has_fresh_gyro()) {
      slip_readings_ = disagrees_with_gyro(turn_radps, turn_variance)
                           ? slip_readings_ + 1
                           : 0;
      if (slip_readings_ >= kSlipReadings) {
        return false;
      }
    }
    update(row(kTurn), turn_radps, turn_variance, is_offset_learnable());
    return true;
  }

  // The gyro reads the turn rate plus its zero offset.
  void update_gyro(double turn_radps, double variance) {
    update(row(kTurn) + row(kGyroBias), turn_radps, variance,
           is_offset_learnable());
    last_gyro_ = GyroReading{*time_s_, turn_radps, variance};
  }

  const Vector& state() const { return state_; }
  const Matrix& covariance() const { return covariance_; }

 private:
  using Row = Eigen::Matrix<double, 1, kSize>;

  struct GyroReading {
    double time_s;
    double turn_radps;
    double variance;
  };

  // Wheel turn rates this many spreads from the gyro's, for this many readings
  // in a row, are slip: starting and stopping disagree for up to 5.
  static constexpr double kSlipGateSpreads = 4.0;
  static constexpr int kSlipReadings = 8;
  static constexpr double kGyroMaxAgeS = 0.03;
  static constexpr double kMaxPredictS = 1.0;

  static Row row(Index index) { return Row::Unit(index); }
  static double squared(double value) { return value * value; }
  static double wrapped(double angle_rad) {
    return std::remainder(angle_rad, 2.0 * std::numbers::pi);
  }

  bool has_fresh_gyro() const {
    return last_gyro_ && *time_s_ - last_gyro_->time_s <= kGyroMaxAgeS;
  }

  bool disagrees_with_gyro(double turn_radps, double turn_variance) const {
    const double gyro_turn_radps = last_gyro_->turn_radps - state_(kGyroBias);
    const double spread = std::sqrt(turn_variance + last_gyro_->variance +
                                    covariance_(kGyroBias, kGyroBias));
    return std::abs(turn_radps - gyro_turn_radps) > kSlipGateSpreads * spread;
  }

  // While the wheels and the gyro disagree, neither teaches the gyro offset.
  bool is_offset_learnable() const { return slip_readings_ == 0; }

  // One scalar reading. Joseph form keeps the covariance right for any gain,
  // including one that leaves the gyro offset alone.
  void update(const Row& h, double reading, double variance,
              bool can_change_gyro_bias = true) {
    const double innovation = reading - (h * state_)(0);
    const double innovation_variance =
        (h * covariance_ * h.transpose())(0) + variance;
    Vector gain = covariance_ * h.transpose() / innovation_variance;
    if (!can_change_gyro_bias) {
      gain(kGyroBias) = 0.0;
    }
    state_ += gain * innovation;
    state_(kYaw) = wrapped(state_(kYaw));
    const Matrix keep = Matrix::Identity() - gain * h;
    covariance_ = keep * covariance_ * keep.transpose() +
                  gain * variance * gain.transpose();
  }

  ProcessNoise noise_;
  Vector state_ = Vector::Zero();
  Matrix covariance_ = Matrix::Zero();
  std::optional<double> time_s_;
  std::optional<GyroReading> last_gyro_;
  int slip_readings_ = 0;
};

}  // namespace robot_ekf
