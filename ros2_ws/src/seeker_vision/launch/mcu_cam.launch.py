from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Camera is on the satellite ESP32 (192.168.8.51); flip 180° because the
    # cam is mounted upside down on our chassis and YOLO regresses on inverted
    # frames. Override --source / --flip on the CLI if the mount changes.
    cam_proxy = ExecuteProcess(
        cmd=["ros2", "run", "seeker_vision", "cam_proxy",
             "--source", "http://192.168.8.51/cam",
             "--flip"],
        output="screen",
    )

    # Delay seeker_vision_node startup so cam_proxy has time to bind its port
    seeker_vision = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="seeker_vision",
                executable="vision_node",
                name="object_detection_node",
                output="screen",
                parameters=[{"video_source": "http://localhost:8080/stream"}],
            )
        ],
    )

    return LaunchDescription([cam_proxy, seeker_vision])
