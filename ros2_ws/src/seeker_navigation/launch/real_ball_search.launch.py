"""Real hardware: full autonomy — EKF + SLAM + Nav2 + ball searcher.

The robot uses the BNO085 IMU for tilt-compensated SLAM, then performs
frontier exploration with Nav2 until it finds and approaches the red ball.

Prerequisites (run before this launch):
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  # Wait for [create_session] log line (ESP32 connected), then launch this.

Usage:
  ros2 launch seeker_navigation real_ball_search.launch.py

Timeline:
  t=0s  robot_state_publisher
  t=2s  EKF starts (fuses /mcu/imu → odom->base_footprint with roll/pitch)
  t=4s  SLAM Toolbox starts
  t=10s SLAM configured + activated
  t=13s Nav2 stack (controller, planner, behavior, bt_navigator, smoother)
  t=15s Nav2 lifecycle manager activates all nodes
  t=25s ball_searcher starts frontier exploration

If Nav2 goal rejections appear after t=25s, SLAM may not be fully active yet.
Check: ros2 lifecycle get /slam_toolbox
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg  = get_package_share_directory('seeker_navigation')
    desc_pkg = get_package_share_directory('seeker_description')

    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    ).toxml()

    slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox_params_real.yaml')
    ekf_params  = os.path.join(nav_pkg, 'config', 'ekf_params.yaml')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params_real.yaml')

    # ── t=0: robot_state_publisher ────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    # ── t=2s: EKF ─────────────────────────────────────────────────────────────
    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_params, {'use_sim_time': False}],
                remappings=[('odometry/filtered', '/odom')],
                output='screen',
            )
        ],
    )

    # ── t=4s: SLAM Toolbox ────────────────────────────────────────────────────
    slam_toolbox = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_params],
                output='screen',
            )
        ],
    )

    slam_activate = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'ros2 lifecycle set /slam_toolbox configure '
                     '&& sleep 2 '
                     '&& ros2 lifecycle set /slam_toolbox activate'],
                output='screen',
            )
        ],
    )

    # ── t=13s: Nav2 nodes ─────────────────────────────────────────────────────
    nav2_nodes = TimerAction(
        period=13.0,
        actions=[
            Node(package='nav2_controller',       executable='controller_server',
                 name='controller_server',         parameters=[nav2_params], output='screen'),
            Node(package='nav2_planner',           executable='planner_server',
                 name='planner_server',            parameters=[nav2_params], output='screen'),
            Node(package='nav2_behaviors',         executable='behavior_server',
                 name='behavior_server',           parameters=[nav2_params], output='screen'),
            Node(package='nav2_bt_navigator',      executable='bt_navigator',
                 name='bt_navigator',              parameters=[nav2_params], output='screen'),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother',         parameters=[nav2_params], output='screen'),
        ],
    )

    # Lifecycle manager delayed 2s after Nav2 nodes to let them register services
    lifecycle_manager = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[nav2_params],
                output='screen',
            )
        ],
    )

    # ── t=25s: ball_searcher ──────────────────────────────────────────────────
    ball_searcher = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='ball_searcher',
                name='ball_searcher',
                parameters=[{'use_sim_time': False}],
                output='screen',
            )
        ],
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav2.rviz')],
        output='screen',
    )

    hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  Real hardware full autonomy starting.\n'
            '  Prerequisites: micro-ROS agent must be running.\n'
            '  t=0s  robot_state_publisher\n'
            '  t=2s  EKF (IMU tilt compensation)\n'
            '  t=4s  SLAM Toolbox\n'
            '  t=10s SLAM activated\n'
            '  t=13s Nav2 stack\n'
            '  t=15s Nav2 lifecycle manager\n'
            '  t=25s ball_searcher (frontier exploration)\n'
            '  RViz Fixed Frame: map\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        robot_state_pub,
        ekf_node,
        slam_toolbox,
        slam_activate,
        nav2_nodes,
        lifecycle_manager,
        ball_searcher,
        rviz,
        hint,
    ])
