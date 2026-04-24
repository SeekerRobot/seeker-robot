"""Simulation: Integrated Medium — Includes YOLO for "Eyes".
Active: Gazebo + Nav2 + SLAM + Seeker + Vision + Muscle.
(Brain/Command node is optional/CLI-driven to save resources).
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
                     'ros2 lifecycle set /slam_toolbox configure '
                     '&& sleep 2 '
                     '&& ros2 lifecycle set /slam_toolbox activate'],
                output='screen',
            )
        ],
    )

    nav2_nodes = TimerAction(
        period=15.0,
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
        period=17.0,
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
        period=30.0,
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

    muscle_node = TimerAction(
        period=35.0,
        actions=[
            Node(
                package="seeker_test_cmd",
                executable="velocity_node",
                name="velocity_node",
                output="screen",
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        fake_mcu,
        ekf_node,
        scan_filter,
        slam_toolbox,
        vision_node,
        slam_activate,
        nav2_nodes,
        lifecycle_manager,
        object_seeker,
        muscle_node,
        rviz,
    ])
