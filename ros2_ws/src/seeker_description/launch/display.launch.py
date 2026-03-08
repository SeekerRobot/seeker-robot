"""Launch robot_state_publisher + RViz for visualisation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory("seeker_description")
    xacro_file = os.path.join(pkg_dir, "urdf", "seeker_hexapod.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(pkg_dir, "rviz", "seeker.rviz"),
                ],
            ),
        ]
    )
