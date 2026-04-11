from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="seeker_tts",
                executable="tts_node",
                name="tts_node",
                output="screen",
                parameters=[
                    {
                        "speaker_url": "http://192.168.1.100:82/speak",
                        "sample_rate": 16000,
                        "fish_model": "s2-pro",
                    }
                ],
            ),
        ]
    )
