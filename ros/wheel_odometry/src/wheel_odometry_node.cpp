// /joint_states to /odom.

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <memory>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "wheel_odometry/diff_drive_odometry.hpp"

namespace wheel_odometry {

// As pico_bridge names them, and the URDF.
constexpr char kLeftJoint[] = "left_wheel_joint";
constexpr char kRightJoint[] = "right_wheel_joint";

// Never observed on a flat floor: very uncertain, so a fusing filter ignores
// them.
constexpr double kUnobservedVariance = 1e6;

// Longer silences: the bridge restarted; the Pico's counts may be back at 0.
constexpr double kNewStreamGapS = 1.0;

// Rows and columns of an odometry covariance.
enum class Axis { kX, kY, kZ, kRoll, kPitch, kYaw };

void set_variance(std::array<double, 36>& covariance, Axis axis,
                  double variance) {
  const auto i = static_cast<size_t>(axis);
  covariance[i * 6 + i] = variance;
}

class WheelOdometryNode : public rclcpp::Node {
 public:
  WheelOdometryNode()
      : Node("wheel_odometry"),
        // No defaults: robot_description's config/robot.yaml is the one copy.
        geometry_{declare_parameter<double>("wheel_diameter_m"),
                  declare_parameter<double>("wheel_separation_m")},
        odom_frame_(declare_parameter("odom_frame", "odom")),
        base_frame_(declare_parameter("base_frame", "base_link")),
        // Unmeasured: a dead-reckoned pose's error grows, so fuse the speeds.
        pose_variance_(declare_parameter("pose_variance", 1e-3)),
        // Encoder rounding while driving (results/README.md, odometry noise).
        linear_velocity_variance_(
            declare_parameter("linear_velocity_variance", 2.2e-6)),
        angular_velocity_variance_(
            declare_parameter("angular_velocity_variance", 3.1e-4)),
        odometry_(geometry_) {
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    joint_state_subscription_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState& msg) {
              on_joint_states(msg);
            });
    RCLCPP_INFO(get_logger(), "Wheel diameter %.4f m, separation %.4f m",
                geometry_.wheel_diameter_m, geometry_.wheel_separation_m);
  }

 private:
  struct Wheel {
    double angle_rad;
    double speed_radps;
  };

  std::optional<Wheel> find_wheel(const sensor_msgs::msg::JointState& msg,
                                  const std::string& joint) const {
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint);
    const size_t index = std::distance(msg.name.begin(), it);
    if (it == msg.name.end() || index >= msg.position.size() ||
        index >= msg.velocity.size()) {
      return std::nullopt;
    }
    return Wheel{msg.position[index], msg.velocity[index]};
  }

  void on_joint_states(const sensor_msgs::msg::JointState& msg) {
    const auto left = find_wheel(msg, kLeftJoint);
    const auto right = find_wheel(msg, kRightJoint);
    if (!left || !right) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "/joint_states lacks %s or %s", kLeftJoint,
                           kRightJoint);
      return;
    }
    if (is_new_wheel_stream(msg.header.stamp)) {
      odometry_.forget_wheel_angles();
    }
    last_stamp_ = msg.header.stamp;
    odometry_.update(left->angle_rad, right->angle_rad);
    publish(
        msg.header.stamp,
        velocity_from_wheels(left->speed_radps, right->speed_radps, geometry_));
  }

  bool is_new_wheel_stream(const rclcpp::Time& stamp) const {
    if (!last_stamp_) {
      return false;
    }
    const double gap_s = (stamp - *last_stamp_).seconds();
    return gap_s < 0.0 || gap_s > kNewStreamGapS;
  }

  void publish(const builtin_interfaces::msg::Time& stamp,
               const Velocity2D& velocity) {
    const Pose2D& pose = odometry_.pose();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.x_m;
    odom.pose.pose.position.y = pose.y_m;
    odom.pose.pose.orientation.z = std::sin(pose.yaw_rad / 2.0);
    odom.pose.pose.orientation.w = std::cos(pose.yaw_rad / 2.0);
    odom.twist.twist.linear.x = velocity.linear_mps;
    odom.twist.twist.angular.z = velocity.angular_radps;
    for (const Axis axis : {Axis::kZ, Axis::kRoll, Axis::kPitch}) {
      set_variance(odom.pose.covariance, axis, kUnobservedVariance);
      set_variance(odom.twist.covariance, axis, kUnobservedVariance);
    }
    for (const Axis axis : {Axis::kX, Axis::kY, Axis::kYaw}) {
      set_variance(odom.pose.covariance, axis, pose_variance_);
    }
    set_variance(odom.twist.covariance, Axis::kX, linear_velocity_variance_);
    // Sideways speed, always ~0.
    set_variance(odom.twist.covariance, Axis::kY, linear_velocity_variance_);
    set_variance(odom.twist.covariance, Axis::kYaw, angular_velocity_variance_);
    odom_publisher_->publish(odom);
  }

  DriveGeometry geometry_;
  std::string odom_frame_;
  std::string base_frame_;
  double pose_variance_;
  double linear_velocity_variance_;
  double angular_velocity_variance_;
  DiffDriveOdometry odometry_;
  std::optional<rclcpp::Time> last_stamp_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;
};

}  // namespace wheel_odometry

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_odometry::WheelOdometryNode>());
  } catch (const std::exception& error) {
    // E.g. started without config/robot.yaml.
    RCLCPP_FATAL(rclcpp::get_logger("wheel_odometry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
