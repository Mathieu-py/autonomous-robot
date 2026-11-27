"""SLAM Toolbox building a map as the robot drives, with our frames and the
LiDAR's range from config/slam.yaml. Start robot.launch.py first."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
    slam_config = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "config", "slam.yaml"])

    return LaunchDescription([
        # SLAM Toolbox's own launch defaults to simulation time; the robot isn't.
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"slam_params_file": slam_config,
                              "use_sim_time": LaunchConfiguration("use_sim_time")}.items()),
    ])
