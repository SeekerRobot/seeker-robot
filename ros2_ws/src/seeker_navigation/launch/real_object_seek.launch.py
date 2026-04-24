"""Real hardware: full autonomy — EKF + SLAM + Nav2 + YOLO + object_seeker.

Brain-Body action pipeline: seeker_voice's command_node (Action Client) sends
SeekObject goals to object_seeker (Action Server). object_seeker starts in
WANDER mode and auto-explores until a /seek_object goal arrives.

Prerequisites (run before this launch):
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  # Wait for [create_session] log line (ESP32 connected), then launch this.

Usage:
  ros2 launch seeker_navigation real_object_seek.launch.py

Timeline:
  t=0s  robot_state_publisher, cam_proxy (ESP32 satellite MJPEG → localhost)
  t=2s  EKF (fuses /mcu/imu → odom→base_footprint with roll/pitch)
        + vision_node (YOLO publisher on /vision/detections)
  t=4s  SLAM Toolbox
  t=10s SLAM configured + activated
  t=13s Nav2 stack (controller, planner, behavior, bt_navigator, smoother)
  t=15s Nav2 lifecycle manager activates all nodes
  t=25s object_seeker (WANDER → frontier exploration; responds to /seek_object)

Trigger a seek (from another terminal):
  ros2 run seeker_navigation find sports_ball --feedback
  ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \\
    "{class_name: 'sports ball', timeout_sec: 60.0}" --feedback
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # When false, object_seeker sits idle after Nav2 comes up and only moves
    # when a /seek_object action goal arrives. When true (default), it auto-
    # starts INITIAL_ROTATION + frontier exploration on boot.
    auto_wander_arg = DeclareLaunchArgument(
        'auto_wander', default_value='true',
        description='If false, object_seeker stays idle until a /seek_object goal arrives.'
    )

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

    # ── t=0: cam_proxy (ESP32 satellite camera → localhost MJPEG) ────────────
    # Camera is upside down on the chassis; YOLO regresses on inverted frames,
    # so --flip rotates each JPEG 180° before re-serving.
    cam_proxy = ExecuteProcess(
        cmd=['ros2', 'run', 'seeker_vision', 'cam_proxy',
             '--source', 'http://192.168.8.51/cam',
             '--flip'],
        output='screen',
    )

    # ── t=2s: EKF + vision_node ──────────────────────────────────────────────
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

    # scan_tilt_filter — drops rays that bounce off floor/ceiling due to body
    # rock during the gait. SLAM + Nav2 costmaps consume the filtered output
    # on /mcu/scan_filtered. Started alongside EKF since it needs odom→base
    # TF to compute body tilt.
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

    # Retry loop: slam_toolbox's lifecycle services can take 10-15s to appear
    # on slower hosts / after a SLAM reset. A single-shot lifecycle call
    # used to race and fail with "Node not found", bricking the launch.
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

    # ── t=13s: Nav2 nodes ─────────────────────────────────────────────────────
    # Topic wiring: every velocity source publishes to /cmd_vel_nav; the
    # velocity_smoother is the sole publisher on /cmd_vel (→ MCU). Prevents
    # the controller_server + object_seeker double-publisher servo jitter.
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
            # cmd_vel_restrict sits between velocity_smoother and the MCU.
            # Enforces {no strafing, no simultaneous forward+turn}. When both
            # axes are commanded, angular wins (robot stops to turn in place).
            # This is the single choke point for scan-match-friendly motion.
            Node(package='seeker_navigation', executable='cmd_vel_restrict',
                 name='cmd_vel_restrict',
                 remappings=[('~/input', 'cmd_vel_smooth'),
                             ('~/output', 'cmd_vel')],
                 output='screen'),
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

    # velocity_smoother is not in lifecycle_manager_navigation's node_names —
    # autostart raced its service registration and left it INACTIVE, which
    # blocks all /cmd_vel output. Configure + activate it explicitly here
    # (same pattern as slam_activate). The `|| true` suffix makes each call
    # idempotent so re-launches against an already-active smoother don't error.
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

    # ── t=25s: object_seeker (Action Server for /seek_object) ─────────────────
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
            '  Real hardware object-seeker pipeline starting.\n'
            '  Prerequisites: micro-ROS agent must be running.\n'
            '  t=0s  robot_state_publisher + cam_proxy (satellite, flipped)\n'
            '  t=2s  EKF (IMU tilt compensation) + YOLO vision_node\n'
            '  t=4s  SLAM Toolbox\n'
            '  t=10s SLAM activated\n'
            '  t=13s Nav2 stack\n'
            '  t=15s Nav2 lifecycle manager\n'
            '  t=18s velocity_smoother configure + activate (explicit)\n'
            '  t=25s object_seeker (WANDER + /seek_object action server)\n'
            '  RViz Fixed Frame: map\n'
            '\n'
            '  Seek a target (friendly CLI):\n'
            '    ros2 run seeker_navigation find sports_ball --feedback\n'
            '    ros2 run seeker_navigation find teddy_bear --timeout 60 -f\n'
            '  Raw action (equivalent):\n'
            "    ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \\\n"
            "      \"{class_name: 'sports ball', timeout_sec: 60.0}\" --feedback\n"
            '  Return to wander:\n'
            '    ros2 service call /wander std_srvs/srv/Trigger\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        auto_wander_arg,
        robot_state_pub,
        cam_proxy,
        ekf_node,
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
