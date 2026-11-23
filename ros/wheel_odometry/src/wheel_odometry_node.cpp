// /joint_states to /odom and the odom -> base_link transform.

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "wheel_odometry/diff_drive_odometry.hpp"

namespace wheel_odometry {

// As pico_bridge names them, and the URDF.
constexpr char kLeftJoint[] = "left_wheel_joint";
constexpr char kRightJoint[] = "right_wheel_joint";

// Never observed on a flat floor: very uncertain, so a fusing filter ignores
// them.
constexpr double kUnobservedVariance = 1e6;

class WheelOdometryNode : public rclcpp::Node {
 public:
  WheelOdometryNode()
      : Node("wheel_odometry"),
        // No defaults: robot_description's config/robot.yaml is the one copy.
        geometry_{declare_parameter<double>("wheel_diameter_m"),
                  declare_parameter<double>("wheel_separation_m")},
        odom_frame_(declare_parameter("odom_frame", "odom")),
        base_frame_(declare_parameter("base_frame", "base_link")),
        // Off once the EKF publishes this transform.
        should_publish_tf_(declare_parameter("publish_tf", true)),
        // Placeholders until step 4 measures them.
        pose_variance_(declare_parameter("pose_variance", 1e-3)),
        linear_velocity_variance_(
            declare_parameter("linear_velocity_variance", 1e-4)),
        angular_velocity_variance_(
            declare_parameter("angular_velocity_variance", 1e-3)),
        odometry_(geometry_) {
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    if (should_publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
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
    odometry_.update(left->angle_rad, right->angle_rad);
    publish(
        msg.header.stamp,
        velocity_from_wheels(left->speed_radps, right->speed_radps, geometry_));
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
    // Diagonals of 6x6 matrices ordered x, y, z, roll, pitch, yaw.
    for (const size_t i : {2, 3, 4}) {
      odom.pose.covariance[i * 7] = kUnobservedVariance;
      odom.twist.covariance[i * 7] = kUnobservedVariance;
    }
    for (const size_t i : {0, 1, 5}) {
      odom.pose.covariance[i * 7] = pose_variance_;
    }
    odom.twist.covariance[0] = linear_velocity_variance_;
    odom.twist.covariance[7] =
        linear_velocity_variance_;  // sideways, always ~0
    odom.twist.covariance[35] = angular_velocity_variance_;
    odom_publisher_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.x_m;
      transform.transform.translation.y = pose.y_m;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  DriveGeometry geometry_;
  std::string odom_frame_;
  std::string base_frame_;
  bool should_publish_tf_;
  double pose_variance_;
  double linear_velocity_variance_;
  double angular_velocity_variance_;
  DiffDriveOdometry odometry_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
