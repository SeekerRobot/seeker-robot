"""Real hardware: scripted /cmd_vel replay against dead-reckoning + SLAM.

Trimmed version of real_object_seek_no_gyro for "what-if" runs: brings up
just enough to produce a usable /odom and a SLAM map, then replays a YAML
script of forward/turn commands on /cmd_vel via `scripted_cmd_vel`.

No Nav2, no velocity_smoother, no cmd_vel_restrict, no object_seeker, no
vision — the script is the sole publisher on /cmd_vel. Turns close the
loop on /odom yaw (from dead_reckoning_odom); forward/backward legs can be
time- or distance-based (see the script YAML).

Prerequisites (run before this launch):
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

Usage:
  ros2 launch seeker_navigation real_scripted_drive.launch.py \\
      script_path:=/path/to/script.yaml \\
      scale:=1.0

Args:
  script_path    YAML path for scripted_cmd_vel (default: packaged example)
  scale          Global multiplier on all durations/distances (default 1.0)
  linear_speed   m/s (default 0.10)
  angular_speed  rad/s (default 0.20)
  rviz           'true' | 'false' (default 'true')
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg  = get_package_share_directory('seeker_navigation')
    desc_pkg = get_package_share_directory('seeker_description')

    default_script = os.path.join(nav_pkg, 'config', 'scripted_cmd_vel_example.yaml')

    script_path_arg   = DeclareLaunchArgument('script_path',   default_value=default_script)
    scale_arg         = DeclareLaunchArgument('scale',         default_value='1.0')
    linear_speed_arg  = DeclareLaunchArgument('linear_speed',  default_value='0.10')
    angular_speed_arg = DeclareLaunchArgument('angular_speed', default_value='0.20')
    rviz_arg          = DeclareLaunchArgument('rviz',          default_value='true')

    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    ).toxml()

    slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox_params_real.yaml')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    # Dead-reckoning /odom is what scripted_cmd_vel closes the turn/distance
    # loops on. Drift is corrected by SLAM's map→odom TF.
    dead_reckoning_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='dead_reckoning_odom',
                name='dead_reckoning_odom',
                parameters=[{'use_sim_time': False}],
                output='screen',
            )
        ],
    )

    scan_inflate_node = Node(
        package='seeker_navigation',
        executable='scan_inflate_node',
        name='scan_inflate',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    scan_tilt_filter_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='scan_tilt_filter',
                name='scan_tilt_filter',
                parameters=[{'use_sim_time': False}],
                output='screen',
            )
        ],
    )

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

    # Script starts after SLAM is up so the first /odom yaw is trustworthy
    # and the map is being built as the robot moves.
    scripted_drive = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='scripted_cmd_vel',
                name='scripted_cmd_vel',
                parameters=[{
                    'use_sim_time': False,
                    'script_path':   LaunchConfiguration('script_path'),
                    'scale':         LaunchConfiguration('scale'),
                    'linear_speed':  LaunchConfiguration('linear_speed'),
                    'angular_speed': LaunchConfiguration('angular_speed'),
                }],
                output='screen',
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  Scripted /cmd_vel drive (NO Nav2 / NO gyro / NO autonomy).\n'
            '  Odom source: dead_reckoning_odom (open-loop /cmd_vel).\n'
            '  scripted_cmd_vel owns /cmd_vel — starts at t=15s.\n'
            '  Prerequisites: micro-ROS agent must be running.\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        script_path_arg,
        scale_arg,
        linear_speed_arg,
        angular_speed_arg,
        rviz_arg,
        robot_state_pub,
        dead_reckoning_node,
        scan_inflate_node,
        scan_tilt_filter_node,
        slam_toolbox,
        slam_activate,
        scripted_drive,
        rviz,
        hint,
    ])
