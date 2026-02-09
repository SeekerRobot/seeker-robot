from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Basic launch file to run the test_package test_node
    # Adjust parameters and namespace as needed later.
    return LaunchDescription([
        Node(
            package="test_package",
            executable="test_node",
            name="test_node",
            output="screen",
        )
    ])
