"""Real hardware: robot_state_publisher + EKF + SLAM Toolbox (IMU tilt compensation).

The EKF fuses /mcu/imu (BNO085 game rotation vector) to produce
odom→base_footprint with correct roll/pitch. SLAM Toolbox traces the full TF chain
(map→odom→base_footprint→base_link→laser) to un-tilt scan rays in the map frame.

This is the recommended mode for real hardware deployment. The tilt compensation
prevents walls from appearing warped when the robot body rolls/pitches while walking.

Prerequisites (run before this launch):
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  # Wait for [create_session] log line (ESP32 connected), then launch this.

Usage:
  ros2 launch seeker_navigation real_slam_ekf.launch.py

Timeline:
  t=0s  robot_state_publisher
  t=2s  EKF starts (fuses /mcu/imu → odom->base_footprint with roll/pitch)
  t=4s  SLAM Toolbox starts (needs EKF TF live first)
  t=10s SLAM configured + activated

Verify tilt compensation:
  ros2 run tf2_ros tf2_echo odom base_footprint
  # Tilt the robot — roll/pitch values should change in real time
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

    # ── t=0: robot_state_publisher ────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    # ── t=2s: EKF ─────────────────────────────────────────────────────────────
    # use_sim_time injected here; ekf_params.yaml intentionally has no field for it
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

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'slam.rviz')],
        output='screen',
    )

    hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  Real hardware SLAM + EKF (IMU tilt compensation) starting.\n'
            '  EKF fuses /mcu/imu (BNO085) → odom->base_footprint.\n'
            '  SLAM uses TF chain to project tilted scans correctly.\n'
            '  Map appears in RViz after ~12s.\n'
            '  Verify tilt compensation:\n'
            '    ros2 run tf2_ros tf2_echo odom base_footprint\n'
            '    (tilt the robot — roll/pitch should change)\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        robot_state_pub,
        ekf_node,
        slam_toolbox,
        slam_activate,
        rviz,
        hint,
    ])
