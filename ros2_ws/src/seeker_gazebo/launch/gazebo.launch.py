"""Launch Gazebo Harmonic with the Seeker hexapod and ROS 2 sensor bridges."""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_pkg = get_package_share_directory("seeker_description")
    gz_pkg = get_package_share_directory("seeker_gazebo")
    ros_gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    # Process xacro → URDF string
    xacro_file = os.path.join(desc_pkg, "urdf", "seeker_hexapod.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    world_file = os.path.join(gz_pkg, "worlds", "slam_test.sdf")
    bridge_config = os.path.join(gz_pkg, "config", "bridge.yaml")
    slam_params_file = os.path.join(gz_pkg, "config", "slam_toolbox_params.yaml")

    # 1. Start Gazebo Harmonic with the SLAM test world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # 2. Spawn the hexapod robot into Gazebo
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "seeker_hexapod",
            "-topic", "/robot_description",
            "-x", "0", "-y", "0", "-z", "0.15",
        ],
        output="screen",
    )

    # 3. Robot state publisher (broadcasts URDF TF tree; handles fixed joints
    #    automatically — joint_state_publisher is not needed)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # 4. ros_gz_bridge — sensor topics, odometry, and cmd_vel from Gazebo/ROS 2
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # 5. Dynamic odom → base_footprint TF from Gazebo odometry.
    #    Replaces the old static_transform_publisher with ground-truth odometry.
    gz_odom_bridge = Node(
        package="seeker_gazebo",
        executable="gz_odom_bridge.py",
        name="gz_odom_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 6. Image transport: republish raw camera image as compressed
    image_republish = Node(
        package="image_transport",
        executable="republish",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "/camera/image"),
            ("out/compressed", "/camera/image/compressed"),
        ],
        output="screen",
    )

    nodes = [
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        gz_odom_bridge,
        image_republish,
    ]

    # 7. slam_toolbox — async online mapping (lifecycle node).
    #    In ROS 2 Jazzy slam_toolbox is a lifecycle node that must be
    #    configured and activated before it starts processing scans.
    try:
        get_package_share_directory("slam_toolbox")
        nodes.append(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_params_file, {"use_sim_time": True}],
                output="screen",
            )
        )
        # Configure → activate the lifecycle node after everything is up
        nodes.append(
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash", "-c",
                            "ros2 lifecycle set /slam_toolbox configure && "
                            "ros2 lifecycle set /slam_toolbox activate",
                        ],
                        output="screen",
                    )
                ],
            )
        )
    except PackageNotFoundError:
        nodes.append(
            LogInfo(msg="slam_toolbox not installed — skipping. "
                        "Install with: sudo apt install ros-jazzy-slam-toolbox")
        )

    return LaunchDescription(nodes)
