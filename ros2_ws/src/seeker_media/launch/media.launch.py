from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="seeker_media",
                executable="mp4_player",
                name="mp4_player",
                output="screen",
                parameters=[
                    {
                        "serve_port": 8383,
                        "threshold": 127,
                        "audio_lead_ms": 200,
                    }
                ],
            ),
        ]
    )
