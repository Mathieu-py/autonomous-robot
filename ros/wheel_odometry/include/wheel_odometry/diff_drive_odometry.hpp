#pragma once

#include <cmath>
#include <numbers>
#include <optional>

// Pure dead reckoning for a two-wheel robot; no ROS.
namespace wheel_odometry {

struct DriveGeometry {
  double wheel_diameter_m;
  double wheel_separation_m;

  double wheel_radius_m() const { return wheel_diameter_m / 2.0; }
};

struct Pose2D {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;  // counterclockwise positive, in (-pi, pi]
};

struct Velocity2D {
  double linear_mps;
  double angular_radps;  // counterclockwise positive
};

// Wheel speeds in rad/s, forward positive on both wheels.
inline Velocity2D velocity_from_wheels(double left_radps, double right_radps,
                                       const DriveGeometry& geometry) {
  const double left_mps = left_radps * geometry.wheel_radius_m();
  const double right_mps = right_radps * geometry.wheel_radius_m();
  return {(left_mps + right_mps) / 2.0,
          (right_mps - left_mps) / geometry.wheel_separation_m};
}

class DiffDriveOdometry {
 public:
  explicit DiffDriveOdometry(const DriveGeometry& geometry)
      : geometry_(geometry) {}

  // Wheel angles in radians, forward positive. The first call only sets the
  // starting angles, so the pose starts at zero wherever the wheels are.
  void update(double left_angle_rad, double right_angle_rad) {
    if (last_angles_) {
      const double wheel_radius_m = geometry_.wheel_radius_m();
      move_along_arc(
          (left_angle_rad - last_angles_->left_rad) * wheel_radius_m,
          (right_angle_rad - last_angles_->right_rad) * wheel_radius_m);
    }
    last_angles_ = WheelAngles{left_angle_rad, right_angle_rad};
  }

  // The next update only sets the starting angles; the pose is kept.
  void forget_wheel_angles() { last_angles_.reset(); }

  const Pose2D& pose() const { return pose_; }

 private:
  struct WheelAngles {
    double left_rad;
    double right_rad;
  };

  // Exact for a constant-curvature step, so turning doesn't drift outward.
  void move_along_arc(double left_m, double right_m) {
    const double distance = (left_m + right_m) / 2.0;
    const double turn = (right_m - left_m) / geometry_.wheel_separation_m;
    const double yaw = pose_.yaw_rad;
    if (std::abs(turn) < 1e-9) {
      pose_.x_m += distance * std::cos(yaw);
      pose_.y_m += distance * std::sin(yaw);
    } else {
      const double turn_radius_m = distance / turn;
      pose_.x_m += turn_radius_m * (std::sin(yaw + turn) - std::sin(yaw));
      pose_.y_m -= turn_radius_m * (std::cos(yaw + turn) - std::cos(yaw));
    }
    pose_.yaw_rad = std::remainder(yaw + turn, 2.0 * std::numbers::pi);
  }

  DriveGeometry geometry_;
  Pose2D pose_;
  std::optional<WheelAngles> last_angles_;
};

}  // namespace wheel_odometry
