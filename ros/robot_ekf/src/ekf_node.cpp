// /odom's speeds and /imu/data_raw's turn rate into /ekf/odom and the
// odom -> base_link transform, with the gyro's estimated zero offset on
// /ekf/gyro_bias. Logs wheel slip and time jumps.

#include <cmath>
#include <memory>
#include <string>
#include <variant>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_ekf/ekf.hpp"
#include "robot_ekf/reorder_buffer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace robot_ekf {

class EkfNode : public rclcpp::Node {
 public:
  EkfNode()
      : Node("robot_ekf"),
        ekf_(
            {.accel_mps2 = declare_parameter("accel_noise_mps2",
                                             kProcessNoise.accel_mps2),
             .yaw_accel_radps2 = declare_parameter(
                 "yaw_accel_noise_radps2", kProcessNoise.yaw_accel_radps2),
             .gyro_bias_drift_radps = declare_parameter(
                 "gyro_bias_drift_radps", kProcessNoise.gyro_bias_drift_radps)},
            declare_parameter("initial_gyro_bias_spread_radps",
                              kInitialGyroBiasSpreadRadps)),
        readings_(kReorderDelayS),
        odom_frame_(declare_parameter("odom_frame", "odom")),
        base_frame_(declare_parameter("base_frame", "base_link")),
        tf_broadcaster_(*this) {
    estimate_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>("ekf/odom", 10);
    gyro_bias_publisher_ =
        create_publisher<std_msgs::msg::Float64>("ekf/gyro_bias", 10);
    wheels_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        [this](const nav_msgs::msg::Odometry& msg) { on_wheels(msg); });
    gyro_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10,
        [this](const sensor_msgs::msg::Imu& msg) { on_gyro(msg); });
  }

 private:
  // Covariance indexes in a 6x6 ordered x, y, z, roll, pitch, yaw.
  static constexpr size_t kXX = 0, kXY = 1, kXYaw = 5, kYX = 6, kYY = 7,
                          kYYaw = 11, kZZ = 14, kRollRoll = 21,
                          kPitchPitch = 28, kYawX = 30, kYawY = 31,
                          kYawYaw = 35;
  // As wheel_odometry: z, roll and pitch aren't estimated.
  static constexpr double kUnobservedVariance = 1e6;
  // Readings wait for both sensors; this long if one goes quiet.
  static constexpr double kReorderDelayS = 0.03;
  // Further back than this is a replay restarting or the clock stepping back.
  static constexpr double kTimeJumpS = 1.0;

  struct StampedWheels {
    builtin_interfaces::msg::Time stamp;
    WheelReading wheels;
  };
  using Reading = std::variant<StampedWheels, GyroReading>;
  using Readings = ReorderBuffer<Reading, std::variant_size_v<Reading>>;

  void on_wheels(const nav_msgs::msg::Odometry& msg) {
    const auto& twist = msg.twist;
    queue(msg.header.stamp,
          StampedWheels{msg.header.stamp,
                        {twist.twist.linear.x, twist.twist.angular.z,
                         twist.covariance[kXX], twist.covariance[kYawYaw]}});
  }

  void on_gyro(const sensor_msgs::msg::Imu& msg) {
    queue(msg.header.stamp, GyroReading{msg.angular_velocity.z,
                                        msg.angular_velocity_covariance[8]});
  }

  void queue(const builtin_interfaces::msg::Time& stamp,
             const Reading& reading) {
    const double t_s = rclcpp::Time(stamp).seconds();
    if (t_s < readings_.newest_s() - kTimeJumpS) {
      RCLCPP_WARN(get_logger(), "Time went back %.1f s: restarting the EKF",
                  readings_.newest_s() - t_s);
      ekf_.restart();
      readings_.clear();
    }
    if (!readings_.add(reading.index(), t_s, reading)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Dropped a reading that arrived too late");
    }
    readings_.release([this](double reading_s, const Reading& oldest) {
      apply(reading_s, oldest);
    });
  }

  void apply(double t_s, const Reading& reading) {
    ekf_.predict_to(t_s);
    if (const auto* gyro = std::get_if<GyroReading>(&reading)) {
      ekf_.update_gyro(*gyro);
      return;
    }
    const auto& stamped = std::get<StampedWheels>(reading);
    if (!ekf_.update_wheels(stamped.wheels)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Wheels slipping: turning by the gyro alone");
    }
    publish(stamped.stamp);
  }

  // Once per wheel reading, 50 Hz.
  void publish(const builtin_interfaces::msg::Time& stamp) {
    const Ekf::Vector& s = ekf_.state();
    const Ekf::Matrix& p = ekf_.covariance();

    nav_msgs::msg::Odometry estimate;
    estimate.header.stamp = stamp;
    estimate.header.frame_id = odom_frame_;
    estimate.child_frame_id = base_frame_;
    estimate.pose.pose.position.x = s(Ekf::kX);
    estimate.pose.pose.position.y = s(Ekf::kY);
    estimate.pose.pose.orientation.z = std::sin(s(Ekf::kYaw) / 2.0);
    estimate.pose.pose.orientation.w = std::cos(s(Ekf::kYaw) / 2.0);
    estimate.twist.twist.linear.x = s(Ekf::kSpeed);
    estimate.twist.twist.angular.z = s(Ekf::kTurn);

    auto& pose = estimate.pose.covariance;
    pose[kXX] = p(Ekf::kX, Ekf::kX);
    pose[kXY] = pose[kYX] = p(Ekf::kX, Ekf::kY);
    pose[kYY] = p(Ekf::kY, Ekf::kY);
    pose[kXYaw] = pose[kYawX] = p(Ekf::kX, Ekf::kYaw);
    pose[kYYaw] = pose[kYawY] = p(Ekf::kY, Ekf::kYaw);
    pose[kYawYaw] = p(Ekf::kYaw, Ekf::kYaw);
    auto& twist = estimate.twist.covariance;
    twist[kXX] = p(Ekf::kSpeed, Ekf::kSpeed);
    twist[kYY] = p(Ekf::kSpeed, Ekf::kSpeed);  // sideways speed, always 0
    twist[kXYaw] = twist[kYawX] = p(Ekf::kSpeed, Ekf::kTurn);
    twist[kYawYaw] = p(Ekf::kTurn, Ekf::kTurn);
    for (const size_t i : {kZZ, kRollRoll, kPitchPitch}) {
      pose[i] = twist[i] = kUnobservedVariance;
    }
    estimate_publisher_->publish(estimate);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = estimate.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = s(Ekf::kX);
    transform.transform.translation.y = s(Ekf::kY);
    transform.transform.rotation = estimate.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);

    std_msgs::msg::Float64 gyro_bias;
    gyro_bias.data = s(Ekf::kGyroBias);
    gyro_bias_publisher_->publish(gyro_bias);
  }

  Ekf ekf_;
  Readings readings_;
  std::string odom_frame_;
  std::string base_frame_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimate_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gyro_bias_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheels_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_subscription_;
};

}  // namespace robot_ekf

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_ekf::EkfNode>());
  rclcpp::shutdown();
  return 0;
}
