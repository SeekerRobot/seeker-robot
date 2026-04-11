"""Simulation: Gazebo + fake MCU + EKF + SLAM Toolbox (IMU-fused odometry).

The EKF fuses /mcu/imu (bridged from Gazebo IMU plugin) to produce
odom→base_footprint with roll/pitch. SLAM Toolbox traces the full TF chain
(map→odom→base_footprint→base_link→laser) to un-tilt scan rays in the map frame.

NOTE: gz_odom_bridge is intentionally OMITTED. Both gz_odom_bridge and ekf_node
publish odom→base_footprint — running both simultaneously causes TF conflicts.
The EKF is the sole odom source here.

This mode tests the exact same pipeline as real_slam_ekf.launch.py but in sim.
Use it to validate EKF+SLAM integration before deploying to real hardware.

Usage:
  ros2 launch seeker_gazebo sim_slam_ekf.launch.py

Timeline:
  t=0s  Gazebo + fake MCU + ROS bridges
  t=2s  EKF starts (fuses /mcu/imu → odom->base_footprint)
  t=5s  SLAM Toolbox starts (needs EKF TF to be live first)
  t=10s SLAM configured + activated
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
    # use_sim_time injected here; ekf_params.yaml intentionally has no field for it
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

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(gz_pkg, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  Sim SLAM + EKF (IMU-fused odometry) starting.\n'
            '  EKF fuses /mcu/imu → odom->base_footprint with roll/pitch.\n'
            '  SLAM uses TF chain to compensate for tilted LiDAR rays.\n'
            '  Map appears in RViz after ~12s.\n'
            '  Drive:\n'
            '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
            '  Verify TF roll/pitch:\n'
            '    ros2 run tf2_ros tf2_echo odom base_footprint\n'
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
        rviz,
        hint,
    ])
