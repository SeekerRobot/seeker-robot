from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "linear_speed",
                default_value="0.5",
                description="Target linear speed in meters per second",
            ),
            Node(
                package="seeker_test_cmd",
                executable="velocity_node",
                name="auto_velocity_node",
                output="screen",
                parameters=[
                    {
                        "drive_mode": "autonomous",
                        "linear_speed": LaunchConfiguration("linear_speed"),
                    }
                ],
            ),
        ]
    )