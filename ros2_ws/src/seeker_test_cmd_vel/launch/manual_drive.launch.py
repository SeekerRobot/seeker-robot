from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="seeker_navigation",  # Assuming a new package name
                executable="velocity_node",   # The name of your compiled code/script
                name="manual_velocity_node",
                output="screen",
                parameters=[
                    {
                        "drive_mode": "manual",
                    }
                ],
            ),
        ]
    )