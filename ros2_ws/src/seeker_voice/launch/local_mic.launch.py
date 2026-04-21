from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="seeker_voice",
                executable="transcription_node",
                name="transcription_node",
                output="screen",
                parameters=[
                    {
                        "audio_source": "local",
                    }
                ],
            ),
            Node(
                package="seeker_voice",
                executable="command_node",
                name="command_node",
                output="screen",
                parameters=[
                    {
                        "gemini_api_key": EnvironmentVariable(
                            "GEMINI_API_KEY", default_value=""
                        ),
                        "gemini_model": "gemini-2.0-flash",
                    }
                ],
            ),
        ]
    )
