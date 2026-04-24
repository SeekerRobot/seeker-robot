"""Real hardware: robot_state_publisher + SLAM Toolbox (no IMU fusion).

Odometry source: none — SLAM Toolbox uses scan-matching only.
Use this for initial bring-up and verifying LiDAR data before adding EKF complexity.
For IMU tilt compensation: ros2 launch seeker_navigation real_slam_ekf.launch.py

Prerequisites (run before this launch):
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  # Wait for [create_session] log line (ESP32 connected), then launch this.

Usage:
  ros2 launch seeker_navigation real_slam_raw.launch.py

Verify:
  ros2 topic hz /mcu/scan        # expect ~10 Hz
  ros2 topic hz /mcu/imu         # expect ~50 Hz
  ros2 topic echo /map --once    # after ~10s
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

    # ── t=0: robot_state_publisher ────────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    # Static identity odom→base_footprint — hexapod has no wheel encoders so
    # there is no physical odometry source in raw mode. SLAM Toolbox requires
    # this TF to exist before it will process any scans. Pure scan-matching
    # will drift over time; use real_slam_ekf.launch.py for IMU-aided odometry.
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    # scan_tilt_filter republishes /mcu/scan → /mcu/scan_filtered (SLAM config
    # points at the filtered topic). With no EKF the tilt filter's roll/pitch
    # lookup returns zero (identity TF), so no rays get dropped — it's a pass-
    # through but needs to run so /mcu/scan_filtered exists for SLAM.
    scan_tilt_filter_node = Node(
        package='seeker_navigation',
        executable='scan_tilt_filter',
        name='scan_tilt_filter',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── t=3s: SLAM Toolbox ────────────────────────────────────────────────────
    # Delayed to allow micro-ROS agent to deliver first /mcu/scan messages
    slam_toolbox = TimerAction(
        period=3.0,
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
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'for i in 1 2 3 4 5 6 7 8 9 10; do '
                     '  ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1 && break; '
                     '  sleep 2; '
                     'done; '
                     'sleep 2; '
                     'for i in 1 2 3 4 5 6 7 8 9 10; do '
                     '  ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1 && break; '
                     '  sleep 2; '
                     'done; '
                     'echo "[slam_activate] slam_toolbox state: "'
                     '  "$(ros2 lifecycle get /slam_toolbox 2>&1)"'],
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
            '  Real hardware SLAM (raw, no IMU fusion) starting.\n'
            '  Prerequisites: micro-ROS agent must be running.\n'
            '  Map will appear in RViz after ~10s.\n'
            '  Verify sensors:\n'
            '    ros2 topic hz /mcu/scan   (expect ~10 Hz)\n'
            '    ros2 topic hz /mcu/imu    (expect ~50 Hz)\n'
            '  For IMU tilt compensation:\n'
            '    ros2 launch seeker_navigation real_slam_ekf.launch.py\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        robot_state_pub,
        static_odom_tf,
        scan_tilt_filter_node,
        slam_toolbox,
        slam_activate,
        rviz,
        hint,
    ])
