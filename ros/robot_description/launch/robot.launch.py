"""The robot's frames, wheel odometry, and two EKFs side by side:
robot_localization's and ours. Ours publishes the odom -> base_link transform.
pico_bridge runs separately, as a service."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_description")
    urdf = Command(["xacro ", PathJoinSubstitution([share, "urdf", "robot.urdf.xacro"])])
    drive_config = PathJoinSubstitution([share, "config", "robot.yaml"])
    ekf_config = PathJoinSubstitution([share, "config", "ekf.yaml"])
    # True when replaying a recording with `ros2 bag play --clock`.
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(urdf, value_type=str)},
                         use_sim_time]),
        Node(package="wheel_odometry", executable="wheel_odometry_node",
             parameters=[drive_config, {"publish_tf": False}, use_sim_time]),
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_filter_node", parameters=[ekf_config, use_sim_time]),
        Node(package="robot_ekf", executable="robot_ekf_node",
             parameters=[use_sim_time]),
    ])
