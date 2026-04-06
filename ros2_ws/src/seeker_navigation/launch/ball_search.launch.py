"""Launch the ball searcher node for the Seeker hexapod.

Run this after `ros2 launch seeker_gazebo gazebo.launch.py` is up.
The robot will execute an expanding square spiral while watching the
camera for the red target ball.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ball_searcher = Node(
        package="seeker_navigation",
        executable="ball_searcher",
        name="ball_searcher",
        output="screen",
    )

    return LaunchDescription([ball_searcher])
