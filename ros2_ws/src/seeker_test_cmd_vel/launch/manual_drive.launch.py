from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="seeker_test_cmd", 
                executable="velocity_node",   
                name="manual_velocity_node",
                output="screen",
                parameters=[
                    {
                        "drive_mode": "manual",
                        "linear_speed": 0.3, # Slower, safer testing speed
                    }
                ],
            ),
        ]
    )