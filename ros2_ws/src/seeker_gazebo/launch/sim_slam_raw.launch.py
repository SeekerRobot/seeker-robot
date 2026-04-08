"""Simulation: Gazebo + fake MCU + SLAM Toolbox (no IMU fusion).

Odometry source: Gazebo ground-truth via gz_odom_bridge (perfect odom).
Use this to verify SLAM works before adding IMU complexity.
For IMU-fused odometry, use sim_slam_ekf.launch.py instead.

Usage:
  ros2 launch seeker_gazebo sim_slam_raw.launch.py

Drive manually in a second terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

What you should see in RViz:
  - /map builds as you drive (~10s after launch)
  - LaserScan overlay matches the map walls
  - Fixed Frame: odom
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
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    ).toxml()

    world_file  = os.path.join(gz_pkg, 'worlds', 'slam_test.sdf')
    bridge_cfg  = os.path.join(gz_pkg, 'config', 'bridge.yaml')
    slam_params = os.path.join(gz_pkg, 'config', 'slam_toolbox_params.yaml')

    # ── Sim infrastructure ────────────────────────────────────────────────────
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

    # gz_odom_bridge provides odom→base_footprint (no EKF in this mode)
    gz_odom_bridge = Node(
        package='seeker_gazebo',
        executable='gz_odom_bridge.py',
        name='gz_odom_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    fake_mcu = Node(
        package='seeker_sim',
        executable='fake_mcu_node',
        name='fake_mcu_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': True}],
        output='screen',
    )

    slam_activate = TimerAction(
        period=8.0,
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
            '  Sim SLAM (raw Gazebo odometry) starting.\n'
            '  Map will appear in RViz after ~10s.\n'
            '  Drive in a second terminal:\n'
            '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
            '  Note: using Gazebo ground-truth odom — no IMU fusion.\n'
            '  For IMU-fused SLAM: ros2 launch seeker_gazebo sim_slam_ekf.launch.py\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        gz_odom_bridge,
        fake_mcu,
        slam_toolbox,
        slam_activate,
        rviz,
        hint,
    ])
