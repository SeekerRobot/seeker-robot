"""Real hardware: full autonomy with VISUAL-COVERAGE-biased exploration.

Identical pipeline to real_object_seek_no_gyro.launch.py / real_object_seek.launch.py
but with two upgrades:

1. Adds visual_coverage_node — maintains a 2D grid of every cell the camera
   has actually pointed through (raycast FOV cone vs. SLAM walls). Published
   on /visual_coverage and reset per-target via a service.

2. object_seeker scores frontiers by (cluster_size + visual_gain_weight ×
   unseen_visual_cells) / distance, and orients each Nav2 goal so the camera
   faces the largest nearby unseen blob. The robot still uses frontier
   exploration as its backbone, but actively *looks* into camera-blind
   regions instead of just driving to LiDAR-blind ones.

Odometry source switches via the `gyro` arg (default false → dead_reckoning_odom,
true → robot_localization EKF fusing /mcu/imu).

Prerequisites:
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

Usage:
  ros2 launch seeker_navigation real_object_seek_thorough.launch.py            # gyro=false
  ros2 launch seeker_navigation real_object_seek_thorough.launch.py gyro:=true # EKF
  ros2 launch seeker_navigation real_object_seek_thorough.launch.py visual_gain_weight:=0.0
      # disable the bias for a pure-frontier baseline (A/B testing)

The two existing launch files (real_object_seek.launch.py,
real_object_seek_no_gyro.launch.py) are intentionally untouched.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gyro_arg = DeclareLaunchArgument(
        'gyro', default_value='false',
        description='If true, use robot_localization EKF fusing /mcu/imu. '
                    'If false (default), use open-loop dead_reckoning_odom.'
    )
    auto_wander_arg = DeclareLaunchArgument(
        'auto_wander', default_value='false',
        description='If false, object_seeker stays idle until a /seek_object goal arrives.'
    )
    visual_gain_weight_arg = DeclareLaunchArgument(
        'visual_gain_weight', default_value='1.0',
        description='How strongly visual-coverage gain biases frontier scoring '
                    '(0 = pure map frontier, no bias).'
    )

    nav_pkg  = get_package_share_directory('seeker_navigation')
    desc_pkg = get_package_share_directory('seeker_description')

    robot_description = xacro.process_file(
        os.path.join(desc_pkg, 'urdf', 'seeker_hexapod.urdf.xacro')
    ).toxml()

    slam_params = os.path.join(nav_pkg, 'config', 'slam_toolbox_params_real.yaml')
    nav2_params = os.path.join(nav_pkg, 'config', 'nav2_params_real.yaml')
    ekf_params  = os.path.join(nav_pkg, 'config', 'ekf_params.yaml')

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

    # ── t=2s: odometry source — exactly one runs based on `gyro` arg ─────────
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
                condition=IfCondition(LaunchConfiguration('gyro')),
            )
        ],
    )

    dead_reckoning_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='dead_reckoning_odom',
                name='dead_reckoning_odom',
                parameters=[{'use_sim_time': False}],
                output='screen',
                condition=UnlessCondition(LaunchConfiguration('gyro')),
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

    # NEW — paints SEEN cells under the camera FOV cone, publishes
    # /visual_coverage. object_seeker reads it for biased frontier scoring
    # and goal-yaw selection. Reset per-SEEK via /visual_coverage/clear.
    visual_coverage_node = Node(
        package='seeker_navigation',
        executable='visual_coverage_node',
        name='visual_coverage',
        parameters=[{
            'use_sim_time': False,
            'fov_rad': 1.396,
            'range_m': 4.0,
        }],
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
                    'visual_gain_weight': LaunchConfiguration('visual_gain_weight'),
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
            '  Real hardware object-seeker (THOROUGH / visual coverage).\n'
            '  Odom source switches on `gyro` arg (default: false = dead-reckoning).\n'
            '  Frontiers are scored by visual-coverage gain in addition to map gain;\n'
            '  set visual_gain_weight:=0.0 to disable for A/B testing.\n'
            '  Coverage map is cleared automatically on every new /seek_object goal.\n'
            '  RViz Fixed Frame: map\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        gyro_arg,
        auto_wander_arg,
        visual_gain_weight_arg,
        robot_state_pub,
        cam_proxy,
        ekf_node,
        dead_reckoning_node,
        scan_inflate_node,
        visual_coverage_node,
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
