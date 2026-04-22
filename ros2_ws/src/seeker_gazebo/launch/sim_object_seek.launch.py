"""Simulation: YOLO-driven object seeker — Gazebo + fake MCU + EKF + SLAM + Nav2
+ gazebo_vision_node + object_seeker.

Self-contained. No other launches required.

object_seeker starts in WANDER mode and auto-explores. Trigger a seek with:
  ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \\
    "{class_name: 'sports ball', timeout_sec: 60.0}" --feedback

Trigger a dance with:
  ros2 service call /perform_move mcu_msgs/srv/PerformMove \\
    "{move_name: 'dance', duration_sec: 3.0}"

Timeline:
  t=0s  Gazebo + fake MCU + ROS bridges
  t=2s  EKF starts (IMU + odom fusion)
  t=5s  SLAM Toolbox + gazebo_vision_node (YOLO)
  t=10s SLAM configured + activated
  t=13s Nav2 stack
  t=15s Nav2 lifecycle manager
  t=25s object_seeker (WANDER → frontier exploration)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
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

    tts_node = Node(
        package='seeker_tts',
        executable='tts_node',
        name='tts_node',
        parameters=[{
            'fish_api_key': EnvironmentVariable('FISH_API_KEY', default_value=''),
            'fish_reference_id': EnvironmentVariable(
                'FISH_REFERENCE_ID', default_value=''
            ),
            'sample_rate': 16000,
            'fish_model': 's2-pro',
        }],
        output='screen',
    )

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

    scan_filter = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='scan_tilt_filter',
                name='scan_tilt_filter',
                parameters=[{'use_sim_time': True, 'max_tilt_rad': 0.06}],
                output='screen',
            )
        ],
    )

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

    vision_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='seeker_vision',
                executable='gazebo_vision_node',
                name='gazebo_vision_node',
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    slam_activate = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'until ros2 node list 2>/dev/null | grep -q slam_toolbox; '
                     'do sleep 1; done '
                     '&& ros2 lifecycle set /slam_toolbox configure '
                     '&& sleep 2 '
                     '&& ros2 lifecycle set /slam_toolbox activate'],
                output='screen',
            )
        ],
    )

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

    object_seeker = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='seeker_navigation',
                executable='object_seeker',
                name='object_seeker',
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    hint = LogInfo(
        msg='\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '  YOLO object seeker demo launching.\n'
            '  t=0s  Gazebo + fake MCU + TTS bridge (:8383/audio_out)\n'
            '  t=2s  EKF\n'
            '  t=5s  SLAM Toolbox + gazebo_vision_node (YOLO)\n'
            '  t=10s SLAM activated\n'
            '  t=13s Nav2 stack\n'
            '  t=15s Nav2 lifecycle manager\n'
            '  t=25s object_seeker (WANDER mode)\n'
            '\n'
            '  Seek a target (friendly CLI):\n'
            '    ros2 run seeker_navigation find teddy_bear --feedback\n'
            '    ros2 run seeker_navigation find sports_ball --timeout 60 -f\n'
            '    ros2 run seeker_navigation find chair\n'
            '  Return to wander:\n'
            '    ros2 service call /wander std_srvs/srv/Trigger\n'
            '  Dance:\n'
            "    ros2 service call /perform_move mcu_msgs/srv/PerformMove \\\n"
            "      \"{move_name: 'dance', duration_sec: 3.0}\"\n"
            '  Raw action (equivalent):\n'
            "    ros2 action send_goal /seek_object mcu_msgs/action/SeekObject \\\n"
            "      \"{class_name: 'sports ball', timeout_sec: 60.0}\" --feedback\n"
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        fake_mcu,
        tts_node,
        ekf_node,
        scan_filter,
        slam_toolbox,
        vision_node,
        slam_activate,
        nav2_nodes,
        lifecycle_manager,
        object_seeker,
        rviz,
        hint,
    ])
