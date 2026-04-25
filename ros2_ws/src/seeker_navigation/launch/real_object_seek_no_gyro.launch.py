"""Real hardware: full autonomy WITHOUT gyro — dead-reckoning odom + SLAM +
Nav2 + YOLO + object_seeker.

Same brain-body pipeline as real_object_seek.launch.py, but the EKF is
replaced with dead_reckoning_odom — an open-loop integrator that turns the
commanded /cmd_vel into an odom→base_footprint TF and a /odom topic. No IMU
is consumed.

Use this to isolate whether SLAM/Nav2 problems are caused by a bad gyro
path. Open-loop odometry drifts quickly on leg slippage; SLAM's map→odom
correction is the only thing that keeps the robot localised. Expect more
frequent scan-match recovery than the EKF variant.

Prerequisites (run before this launch):
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

Usage:
  ros2 launch seeker_navigation real_object_seek_no_gyro.launch.py

Timeline:
  t=0s  robot_state_publisher, cam_proxy
  t=2s  dead_reckoning_odom + scan_tilt_filter + vision_node
  t=4s  SLAM Toolbox
  t=10s SLAM activated
  t=13s Nav2 stack
  t=15s Nav2 lifecycle manager
  t=18s velocity_smoother explicit activate
  t=25s object_seeker
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_wander_arg = DeclareLaunchArgument(
        'auto_wander', default_value='false',
        description='If false, object_seeker stays idle until a /seek_object goal arrives.'
    )

    nav_pkg  = get_package_share_directory('seeker_navigation')
    desc_pkg = get_package_share_directory('seeker_description')

    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    ).toxml()

    slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox_params_real.yaml')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params_real.yaml')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        remappings=[('joint_states', '/mcu/joint_states')],
    )

    cam_proxy = ExecuteProcess(
        cmd=['ros2', 'run', 'seeker_vision', 'cam_proxy',
             '--source', 'http://192.168.8.51/cam',
             '--flip'],
        output='screen',
    )

    # ── t=2s: dead_reckoning_odom (replaces EKF) ──────────────────────────────
    # Integrates /cmd_vel → odom→base_footprint TF + /odom topic. Open-loop,
    # drifts with leg slippage. SLAM's map→odom correction compensates.
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

    vision_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='seeker_vision',
                executable='vision_node',
                name='object_detection_node',
                parameters=[{'video_source': 'http://localhost:8080/stream'}],
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

    nav2_nodes = TimerAction(
        period=13.0,
        actions=[
            Node(package='nav2_controller',       executable='controller_server',
                 name='controller_server',         parameters=[nav2_params],
                 remappings=[('cmd_vel', 'cmd_vel_nav')],
                 output='screen'),
            Node(package='nav2_planner',           executable='planner_server',
                 name='planner_server',            parameters=[nav2_params], output='screen'),
            Node(package='nav2_behaviors',         executable='behavior_server',
                 name='behavior_server',           parameters=[nav2_params], output='screen'),
            Node(package='nav2_bt_navigator',      executable='bt_navigator',
                 name='bt_navigator',              parameters=[nav2_params], output='screen'),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother',         parameters=[nav2_params],
                 remappings=[('cmd_vel', 'cmd_vel_nav'),
                             ('cmd_vel_smoothed', 'cmd_vel_smooth')],
                 output='screen'),
            Node(package='seeker_navigation', executable='cmd_vel_restrict',
                 name='cmd_vel_restrict',
                 remappings=[('~/input', 'cmd_vel_smooth'),
                             ('~/output', 'cmd_vel')],
                 output='screen'),
        ],
    )

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

    vsmoother_activate = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     '(ros2 lifecycle set /velocity_smoother configure || true) '
                     '&& sleep 2 '
                     '&& (ros2 lifecycle set /velocity_smoother activate || true)'],
                output='screen',
            )
        ],
    )

    object_seeker = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='object_seeker',
                name='object_seeker',
                parameters=[{
                    'use_sim_time': False,
                    'auto_wander': LaunchConfiguration('auto_wander'),
                }],
                output='screen',
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav2.rviz')],
        output='screen',
    )

    hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  Real hardware object-seeker pipeline (NO GYRO).\n'
            '  Odom source: open-loop integration of /cmd_vel.\n'
            '  Prerequisites: micro-ROS agent must be running.\n'
            '  Drift-heavy; SLAM scan matching is the ground truth.\n'
            '  RViz Fixed Frame: map\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        auto_wander_arg,
        robot_state_pub,
        cam_proxy,
        dead_reckoning_node,
        scan_inflate_node,
        scan_tilt_filter_node,
        vision_node,
        slam_toolbox,
        slam_activate,
        nav2_nodes,
        lifecycle_manager,
        vsmoother_activate,
        object_seeker,
        rviz,
        hint,
    ])
