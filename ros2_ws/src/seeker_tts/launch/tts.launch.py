from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
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
                        "fish_api_key": EnvironmentVariable("FISH_API_KEY", default_value=""),
                        "fish_reference_id": EnvironmentVariable("FISH_REFERENCE_ID", default_value=""),
                        "sample_rate": 16000,
                        "fish_model": "s2-pro",
                    }
                ],
            ),
        ]
    )
