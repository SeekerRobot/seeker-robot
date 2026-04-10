"""Launch slam_toolbox + Nav2 + ball searcher for the Seeker hexapod.

Run AFTER: ros2 launch seeker_gazebo sim_teleop.launch.py
That launch provides: Gazebo, fake_mcu_node (tripod gait), ros_gz_bridge,
gz_odom_bridge, and robot_state_publisher.

This launch adds:
  - slam_toolbox  (online async SLAM → /map + map→odom TF)
  - Nav2 stack    (controller, planner, behavior, bt_navigator, velocity_smoother)
  - ball_searcher (frontier exploration + approach)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("seeker_navigation")
    gz_pkg  = get_package_share_directory("seeker_gazebo")

    nav2_params       = os.path.join(nav_pkg, "config", "nav2_params.yaml")
    slam_params_file  = os.path.join(gz_pkg,  "config", "slam_toolbox_params.yaml")

    # ── slam_toolbox (lifecycle node) ────────────────────────────────────────
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params_file, {"use_sim_time": True}],
        output="screen",
    )

    # Configure → activate slam_toolbox after it has had time to register its
    # lifecycle services.  A sleep between the two transitions prevents the
    # "No transition matching activate found for current state unconfigured"
    # race condition seen when they are fired back-to-back.
    slam_activate = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c",
                     "ros2 lifecycle set /slam_toolbox configure "
                     "&& sleep 2 "
                     "&& ros2 lifecycle set /slam_toolbox activate"],
                output="screen",
            )
        ],
    )

    # ── Nav2 nodes ────────────────────────────────────────────────────────────
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

    # Delay lifecycle_manager so all Nav2 nodes have registered their lifecycle
    # services before it attempts the configure transition.
    lifecycle_manager = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[nav2_params],
                output="screen",
            )
        ],
    )

    # Ball searcher — wait for Nav2 to be fully active before starting.
    ball_searcher = TimerAction(
        period=15.0,
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
        slam,
        slam_activate,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        ball_searcher,
    ])
