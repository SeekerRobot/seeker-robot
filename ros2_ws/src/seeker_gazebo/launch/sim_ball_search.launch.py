"""Simulation: Full autonomy demo — Gazebo + fake MCU + EKF + SLAM + Nav2 + ball searcher.

Self-contained. No other launches required.

The robot performs an initial 360° rotation to seed the SLAM map, then uses
Nav2 frontier exploration to search for the red ball. When found, it approaches.

Timeline:
  t=0s  Gazebo + fake MCU + ROS bridges
  t=2s  EKF starts (IMU fusion, odom->base_footprint with roll/pitch)
  t=5s  SLAM Toolbox starts
  t=10s SLAM configured + activated
  t=13s Nav2 stack starts (controller, planner, behavior, bt_navigator, smoother)
  t=15s Nav2 lifecycle manager activates all nodes
  t=25s ball_searcher starts frontier exploration

Usage:
  ros2 launch seeker_gazebo sim_ball_search.launch.py

Watch in RViz (Fixed Frame: map):
  - Map builds as robot rotates
  - Green path = Nav2 global plan
  - Blue path  = Nav2 local plan
  - Costmaps appear once Nav2 is active
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_pkg   = get_package_share_directory('seeker_description')
    gz_pkg     = get_package_share_directory('seeker_gazebo')
    nav_pkg    = get_package_share_directory('seeker_navigation')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    ).toxml()

    world_file  = os.path.join(gz_pkg, 'worlds', 'slam_test.sdf')
    bridge_cfg  = os.path.join(gz_pkg, 'config', 'bridge.yaml')
    slam_params = os.path.join(gz_pkg, 'config', 'slam_toolbox_params.yaml')
    ekf_params  = os.path.join(nav_pkg, 'config', 'ekf_params.yaml')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')

    # ── Sim infrastructure (NO gz_odom_bridge — EKF replaces it) ─────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'seeker', '-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0.10'],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen',
    )

    fake_mcu = Node(
        package='seeker_sim',
        executable='fake_mcu_node',
        name='fake_mcu_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── EKF (t=2s) ────────────────────────────────────────────────────────────
    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_params, {'use_sim_time': True}],
                remappings=[('odometry/filtered', '/odom')],
                output='screen',
            )
        ],
    )

    # ── SLAM Toolbox (t=5s) ───────────────────────────────────────────────────
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_params, {'use_sim_time': True}],
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

    # ── Nav2 nodes (t=13s) ────────────────────────────────────────────────────
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

    # ── Ball searcher (t=25s) ─────────────────────────────────────────────────
    ball_searcher = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='ball_searcher',
                name='ball_searcher',
                parameters=[{'use_sim_time': True}],
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
            '  Full autonomy demo launching.\n'
            '  t=0s  Gazebo + fake MCU\n'
            '  t=2s  EKF (IMU fusion)\n'
            '  t=5s  SLAM Toolbox\n'
            '  t=10s SLAM activated\n'
            '  t=13s Nav2 stack\n'
            '  t=15s Nav2 lifecycle manager\n'
            '  t=25s ball_searcher (frontier exploration)\n'
            '  RViz Fixed Frame: map\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        fake_mcu,
        ekf_node,
        slam_toolbox,
        slam_activate,
        nav2_nodes,
        lifecycle_manager,
        ball_searcher,
        rviz,
        hint,
    ])
