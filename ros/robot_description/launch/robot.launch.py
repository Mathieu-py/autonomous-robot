"""The robot's frames and wheel odometry. pico_bridge runs separately, as a service."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_description")
    urdf = Command(["xacro ", PathJoinSubstitution([share, "urdf", "robot.urdf.xacro"])])
    drive_config = PathJoinSubstitution([share, "config", "robot.yaml"])

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}]),
        Node(package="wheel_odometry", executable="wheel_odometry_node",
             parameters=[drive_config]),
    ])
