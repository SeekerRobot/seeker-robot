"""Launch Gazebo Harmonic + Seeker hexapod + fake_mcu_node for teleop testing.

This is the entry point for simulation development. It replaces the physical
ESP32 (MCU) with fake_mcu_node — a Python node that runs the same tripod gait
as GaitController.cpp, publishing /mcu/joint_states and commanding Gazebo joints.

Usage:
  ros2 launch seeker_gazebo sim_teleop.launch.py

Then in a second terminal:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  (i/,  forward/back  |  j/l  rotate  |  k  stop  |  q/z  speed ±)

What you should see:
  - Gazebo opens with the hexapod in a test world
  - When you press i, the hexapod walks with alternating tripod legs
  - /mcu/joint_states publishes at ~100 Hz
  - /mcu/imu publishes at ~200 Hz, /mcu/scan at ~10 Hz

To switch to real hardware:
  1. Stop this launch
  2. Start the micro-ROS agent: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  3. Flash test_bridge_gait to the ESP32
  4. Run seeker_gazebo/launch/sim_teleop.launch.py with use_fake_mcu:=false
     (robot_state_publisher already remaps joint_states → /mcu/joint_states)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_pkg  = get_package_share_directory('seeker_description')
    gz_pkg    = get_package_share_directory('seeker_gazebo')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    # Process xacro → URDF string once at launch time
    xacro_file       = os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    world_file   = os.path.join(gz_pkg, 'worlds', 'slam_test.sdf')
    bridge_cfg   = os.path.join(gz_pkg, 'config', 'bridge.yaml')

    # ── 1. Gazebo Harmonic ────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── 2. Robot state publisher ──────────────────────────────────────────
    # Remaps joint_states → /mcu/joint_states so it reads from fake_mcu_node
    # (or from the real MCU micro-ROS bridge when running on hardware).
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,   # wall time — matches fake_mcu_node timestamps
        }],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    # ── 3. Spawn robot into Gazebo ────────────────────────────────────────
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  'seeker',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.10',  # 10cm clearance — knee joints start at 0° (tibia vertical)
                                              # and need room before settling to NEUTRAL_KNEE=45°
        ],
        output='screen',
    )

    # ── 4. ros_gz_bridge ──────────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
        output='screen',
    )

    # ── 5. Odometry TF bridge (odom → base_footprint) ────────────────────
    gz_odom_bridge = Node(
        package='seeker_gazebo',
        executable='gz_odom_bridge.py',
        name='gz_odom_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 6. fake_mcu_node — tripod gait + joint publisher ─────────────────
    # Subscribes to /cmd_vel, publishes /mcu/joint_states (12 joints)
    # and std_msgs/Float64 to each Gazebo joint command topic.
    fake_mcu = Node(
        package='seeker_sim',
        executable='fake_mcu_node',
        name='fake_mcu_node',
        parameters=[{'use_sim_time': False}],   # wall-clock: starts immediately, no sim-clock race condition
        output='screen',
    )

    drive_hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  Hexapod simulation ready.\n'
            '  Drive with teleop_twist_keyboard in a new terminal:\n'
            '    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n'
            '  i / ,  forward / backward\n'
            '  j / l  rotate left / right\n'
            '  k      stop\n'
            '  q / z  increase / decrease speed\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        gz_odom_bridge,
        fake_mcu,
        drive_hint,
    ])
