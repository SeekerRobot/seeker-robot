from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "esp32_ip",
                default_value="192.168.8.50",
                description="IP address of the ESP32 board",
            ),
            DeclareLaunchArgument(
                "esp32_port",
                default_value="81",
                description="HTTP port for the ESP32 /audio endpoint (MicSubsystem default: 81)",
            ),
            Node(
                package="seeker_voice",
                executable="transcription_node",
                name="transcription_node",
                output="screen",
                parameters=[
                    {
                        "audio_source": "esp32",
                        "esp32_ip": LaunchConfiguration("esp32_ip"),
                        "esp32_port": LaunchConfiguration("esp32_port"),
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
