"""Launch Nav2 + SLAM + ball searcher for real hardware.

Assumes the following are already running (provided by hardware team):
  - /lidar/scan        (sensor_msgs/LaserScan)   — LiDAR driver
  - /odom              (nav_msgs/Odometry)        — odometry source
  - odom -> base_footprint TF                     — odometry TF
  - /camera/image      (sensor_msgs/Image)        — camera driver
  - /cmd_vel subscriber                           — motor controller

Launch order:
  1. Start hardware drivers (team's responsibility)
  2. ros2 launch seeker_navigation real_ball_search.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("seeker_navigation")
    desc_pkg = get_package_share_directory("seeker_description")
    gz_pkg = get_package_share_directory("seeker_gazebo")

    nav2_params = os.path.join(nav_pkg, "config", "nav2_params.yaml")
    slam_params = os.path.join(gz_pkg, "config", "slam_toolbox_params.yaml")

    # URDF for robot_state_publisher
    import xacro
    xacro_file = os.path.join(desc_pkg, "urdf", "seeker_hexapod.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    # -- Robot state publisher (broadcasts URDF fixed joints) --
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # -- slam_toolbox (lifecycle node) --
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": False}],
        output="screen",
    )

    # Configure + activate slam_toolbox after startup
    from launch.actions import ExecuteProcess
    slam_activate = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "ros2 lifecycle set /slam_toolbox configure && "
                    "ros2 lifecycle set /slam_toolbox activate",
                ],
                output="screen",
            )
        ],
    )

    # -- Nav2 nodes (same as simulation, but use_sim_time: false) --
    nav2_common_params = [nav2_params, {"use_sim_time": False}]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=nav2_common_params,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        output="screen",
        parameters=nav2_common_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        output="screen",
        parameters=nav2_common_params,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        output="screen",
        parameters=nav2_common_params,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        output="screen",
        parameters=nav2_common_params,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=nav2_common_params,
        output="screen",
    )

    # -- Ball searcher (delayed to let Nav2 start) --
    ball_searcher = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="seeker_navigation",
                executable="ball_searcher",
                name="ball_searcher",
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        robot_state_pub,
        slam_toolbox,
        slam_activate,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        ball_searcher,
    ])
