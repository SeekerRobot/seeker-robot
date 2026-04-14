from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision = Node(
        package="seeker_vision",
        executable="vision_node",
        name="object_detection_node",
        output="screen",
        parameters=[{"video_source": "/dev/video0"}],
    )
    return LaunchDescription([vision])
