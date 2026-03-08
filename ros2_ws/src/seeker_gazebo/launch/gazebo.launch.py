"""Launch Gazebo Harmonic with the Seeker hexapod and ROS 2 sensor bridges."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    # 4. ros_gz_bridge — sensor topics from Gazebo to ROS 2
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # 5. Static odom → base_footprint transform
    #    Provides the TF frame that slam_toolbox needs.
    #    slam_toolbox uses scan matching as primary alignment, so SLAM
    #    still works when manually moving the robot in small increments.
    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
    )

    # 6. Image transport: republish raw camera image as compressed
    #    so it matches the /camera/image/compressed topic expected by SLAM
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

    return LaunchDescription(
        [
            gz_sim,
            robot_state_pub,
            gz_spawn,
            gz_bridge,
            odom_tf,
            image_republish,
        ]
    )
