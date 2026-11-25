// Bridges /cmd_vel, the Pico's status and its IMU; powers the Pi off on a
// flat battery.

#include <chrono>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "pico_bridge/drive_kinematics.hpp"
#include "pico_bridge/gyro_bias.hpp"
#include "pico_bridge/pico_protocol.hpp"
#include "pico_bridge/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace pico_bridge {

constexpr double kRadiansPerRev = 2.0 * std::numbers::pi;
constexpr double kDegreesPerRadian = 180.0 / std::numbers::pi;
constexpr size_t kGyroBiasWindowSize = 50;         // 1 s at 50 Hz
constexpr double kGyroBiasMaxSpreadRadps = 0.035;  // 2 deg/s
constexpr double kImuSilenceS = 1.0;

class PicoBridgeNode : public rclcpp::Node {
 public:
  PicoBridgeNode()
      : Node("pico_bridge"),
        port_path_(declare_parameter("port", "/dev/pico")),
        // No defaults: robot_description's config/robot.yaml is the one copy.
        geometry_{declare_parameter<double>("wheel_diameter_m"),
                  declare_parameter<double>("wheel_separation_m")},
        motor_mapping_{
            .is_left_first_motor =
                declare_parameter<std::string>("left_motor") == "first",
            .is_left_reversed = declare_parameter<bool>("left_reversed"),
            .is_right_reversed = declare_parameter<bool>("right_reversed")},
        ticks_per_rev_(declare_parameter<double>("ticks_per_rev")),
        cmd_vel_timeout_(rclcpp::Duration::from_seconds(
            declare_parameter("cmd_vel_timeout_s", 1.0))),
        imu_frame_id_(declare_parameter("imu_frame_id", "imu_link")),
        // Largest axis in results/imu/2026-11-23-run01-still-172s.csv.
        gyro_variance_(declare_parameter("gyro_variance", 4.5e-7)),
        accel_variance_(declare_parameter("accel_variance", 5.3e-4)),
        serial_(port_path_),
        gyro_bias_(kGyroBiasWindowSize, kGyroBiasMaxSpreadRadps),
        last_cmd_vel_time_(now()),
        last_imu_time_(now()) {
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist& twist) {
          last_cmd_vel_ = twist;
          last_cmd_vel_time_ = now();
        });
    joint_state_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    battery_publisher_ =
        create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
    imu_publisher_ =
        create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this] { exchange_with_pico(); });
    RCLCPP_INFO(get_logger(), "Connected to the Pico on %s",
                port_path_.c_str());
  }

 private:
  // Every 20 ms: send the command, then handle the Pico's lines.
  void exchange_with_pico() {
    if (!serial_.write_text(command_for_pico()) ||
        !serial_.read_available(received_)) {
      // A reconnected Pico is a new device: exit and let systemd restart us.
      RCLCPP_FATAL(get_logger(), "Lost the serial link to the Pico; exiting");
      rclcpp::shutdown();
      return;
    }
    while (const auto line = take_line(received_)) {
      if (const auto status = parse_status_line(*line)) {
        handle_status(*status);
      } else if (const auto sample = parse_imu_line(*line)) {
        handle_imu(*sample);
      } else if (is_shutdown_request(*line)) {
        power_off_once(*line);
      } else if (!line->empty()) {
        // One-off messages, e.g. a watchdog reboot.
        RCLCPP_WARN(get_logger(), "Pico: %s", line->c_str());
      }
    }
    warn_once_if_imu_silent();
  }

  // Publishes once the gyro offset is measured: keep the robot still at start.
  void handle_imu(const ImuSample& sample) {
    last_imu_time_ = now();
    if (is_imu_silent_) {
      RCLCPP_INFO(get_logger(), "IMU data is back");
      is_imu_silent_ = false;
    }
    if (!gyro_bias_.is_ready()) {
      measure_gyro_bias(sample.gyro_radps);
    }
    if (gyro_bias_.is_ready()) {
      publish_imu(sample);
    }
  }

  void measure_gyro_bias(const Vector3& gyro_radps) {
    switch (gyro_bias_.add(gyro_radps)) {
      case GyroBias::Progress::kMeasuring:
        break;
      case GyroBias::Progress::kMovedSoRestarted:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "The robot moved while measuring the gyro "
                             "offset; keep it still");
        break;
      case GyroBias::Progress::kReady: {
        const Vector3 bias = gyro_bias_.bias();
        RCLCPP_INFO(get_logger(), "Gyro offset (deg/s): %.2f %.2f %.2f",
                    bias.x * kDegreesPerRadian, bias.y * kDegreesPerRadian,
                    bias.z * kDegreesPerRadian);
        break;
      }
    }
  }

  void publish_imu(const ImuSample& sample) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = last_imu_time_;
    imu.header.frame_id = imu_frame_id_;
    imu.orientation_covariance[0] = -1.0;  // no orientation estimate
    const Vector3 gyro = gyro_bias_.remove_from(sample.gyro_radps);
    imu.angular_velocity.x = gyro.x;
    imu.angular_velocity.y = gyro.y;
    imu.angular_velocity.z = gyro.z;
    imu.linear_acceleration.x = sample.accel_mps2.x;
    imu.linear_acceleration.y = sample.accel_mps2.y;
    imu.linear_acceleration.z = sample.accel_mps2.z;
    for (const size_t diagonal : {0, 4, 8}) {
      imu.angular_velocity_covariance[diagonal] = gyro_variance_;
      imu.linear_acceleration_covariance[diagonal] = accel_variance_;
    }
    imu_publisher_->publish(imu);
  }

  void warn_once_if_imu_silent() {
    if (is_imu_silent_ || (now() - last_imu_time_).seconds() <= kImuSilenceS) {
      return;
    }
    is_imu_silent_ = true;
    RCLCPP_WARN(get_logger(), "No IMU data from the Pico for %.0f s",
                kImuSilenceS);
  }

  void power_off_once(const std::string& request) {
    if (is_powering_off_) {
      return;
    }
    is_powering_off_ = true;
    RCLCPP_FATAL(get_logger(), "Pico: %s. Powering off the Pi",
                 request.c_str());
    // -n: fail rather than wait for a password nobody will type.
    const int result = std::system("sudo -n /usr/bin/systemctl poweroff");
    if (result != 0) {
      RCLCPP_ERROR(get_logger(),
                   "Power-off failed (status %d): is the sudoers rule from "
                   "docs/pi-setup.md in place?",
                   result);
    }
  }

  std::string command_for_pico() const {
    const bool is_cmd_vel_fresh =
        now() - last_cmd_vel_time_ <= cmd_vel_timeout_;
    const geometry_msgs::msg::Twist twist =
        is_cmd_vel_fresh ? last_cmd_vel_ : geometry_msgs::msg::Twist();
    const MotorPair motors = motor_mapping_.to_motors(
        wheel_rps_from_twist(twist.linear.x, twist.angular.z, geometry_));
    return format_command(motors.first, motors.second);
  }

  void handle_status(const PicoStatus& status) {
    const rclcpp::Time stamp = now();
    publish_joint_states(status, stamp);
    publish_battery(status.battery_volts, stamp);
    log_stop_change(status.stop_reason);
  }

  void publish_joint_states(const PicoStatus& status,
                            const rclcpp::Time& stamp) {
    const WheelPair counts =
        motor_mapping_.to_wheels({static_cast<double>(status.first_count),
                                  static_cast<double>(status.second_count)});
    const WheelPair rps =
        motor_mapping_.to_wheels({status.first_rps, status.second_rps});

    sensor_msgs::msg::JointState joints;
    joints.header.stamp = stamp;
    joints.name = {"left_wheel_joint", "right_wheel_joint"};
    joints.position = {counts.left / ticks_per_rev_ * kRadiansPerRev,
                       counts.right / ticks_per_rev_ * kRadiansPerRev};
    joints.velocity = {rps.left * kRadiansPerRev, rps.right * kRadiansPerRev};
    joint_state_publisher_->publish(joints);
  }

  void publish_battery(double volts, const rclcpp::Time& stamp) {
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = stamp;
    battery.voltage = static_cast<float>(volts);
    battery.present = true;
    battery_publisher_->publish(battery);
  }

  void log_stop_change(const std::string& stop_reason) {
    if (stop_reason == last_stop_reason_) {
      return;
    }
    if (stop_reason == kNoStopReason) {
      RCLCPP_INFO(get_logger(), "Pico stop cleared");
    } else {
      RCLCPP_WARN(get_logger(), "Pico stopped the motors: %s",
                  stop_reason.c_str());
    }
    last_stop_reason_ = stop_reason;
  }

  std::string port_path_;
  DriveGeometry geometry_;
  MotorMapping motor_mapping_;
  double ticks_per_rev_;
  rclcpp::Duration cmd_vel_timeout_;
  std::string imu_frame_id_;
  double gyro_variance_;
  double accel_variance_;
  SerialPort serial_;
  GyroBias gyro_bias_;
  bool is_imu_silent_ = false;
  std::string received_;
  std::string last_stop_reason_ = kNoStopReason;
  bool is_powering_off_ = false;

  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pico_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<pico_bridge::PicoBridgeNode>());
  } catch (const std::exception& error) {
    // E.g. no Pico at boot; systemd retries.
    RCLCPP_FATAL(rclcpp::get_logger("pico_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
