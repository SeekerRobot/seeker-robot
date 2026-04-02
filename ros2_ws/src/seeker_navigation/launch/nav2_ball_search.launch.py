"""Launch Nav2 + frontier-based ball searcher for the Seeker hexapod.

Run after: ros2 launch seeker_gazebo gazebo.launch.py
This brings up only the Nav2 nodes we need (no collision_monitor,
route_server, docking_server, etc.) plus the ball_searcher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("seeker_navigation")
    nav2_params = os.path.join(nav_pkg, "config", "nav2_params.yaml")

    # -- Nav2 nodes (only what we need) --

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[nav2_params],
        output="screen",
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[nav2_params],
        output="screen",
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[nav2_params],
        output="screen",
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[nav2_params],
        output="screen",
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[nav2_params],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[nav2_params],
        output="screen",
    )

    # Ball searcher with delay to let Nav2 lifecycle nodes fully activate
    ball_searcher = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="seeker_navigation",
                executable="ball_searcher",
                name="ball_searcher",
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        ball_searcher,
    ])
