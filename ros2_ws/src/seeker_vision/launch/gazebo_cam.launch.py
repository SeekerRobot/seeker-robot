from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision = Node(
        package="seeker_vision",
        executable="gazebo_vision_node",
        name="object_detection_node",
        output="screen",
    )
    return LaunchDescription([vision])
