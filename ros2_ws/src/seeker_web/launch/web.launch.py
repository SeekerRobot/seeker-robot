"""
Launch the Seeker web controller.

  ros2 launch seeker_web web.launch.py
  ros2 launch seeker_web web.launch.py mcu_ip:=192.168.1.50 http_port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mcu_ip", default_value="192.168.1.50"),
            DeclareLaunchArgument("http_port", default_value="8080"),
            DeclareLaunchArgument("cam_port", default_value="80"),
            DeclareLaunchArgument("mic_port", default_value="81"),
            DeclareLaunchArgument("lidar_max_points", default_value="180"),
            DeclareLaunchArgument("status_rate_s", default_value="0.1"),
            DeclareLaunchArgument("imu_rate_s", default_value="0.05"),
            DeclareLaunchArgument("lidar_rate_s", default_value="0.2"),
            DeclareLaunchArgument("log_rate_s", default_value="0.1"),
            Node(
                package="seeker_web",
                executable="web_node",
                name="seeker_web",
                output="screen",
                parameters=[
                    {
                        "mcu_ip": LaunchConfiguration("mcu_ip"),
                        "http_port": LaunchConfiguration("http_port"),
                        "cam_port": LaunchConfiguration("cam_port"),
                        "mic_port": LaunchConfiguration("mic_port"),
                        "lidar_max_points": LaunchConfiguration("lidar_max_points"),
                        "status_rate_s": LaunchConfiguration("status_rate_s"),
                        "imu_rate_s": LaunchConfiguration("imu_rate_s"),
                        "lidar_rate_s": LaunchConfiguration("lidar_rate_s"),
                        "log_rate_s": LaunchConfiguration("log_rate_s"),
                        # play_wav_allow_roots keeps its node-level default
                        # list; override via --ros-args -p when exposing this
                        # node off-LAN.
                    }
                ],
            ),
        ]
    )
