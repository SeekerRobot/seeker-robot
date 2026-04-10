#!/usr/bin/env python3
"""Bridge Gazebo odometry to a dynamic odom -> base_footprint TF for slam_toolbox.

The OdometryPublisher Gazebo plugin publishes nav_msgs/Odometry on /odom
with proper sim-time timestamps. This node extracts the pose from that
message and broadcasts it as an odom -> base_footprint TF so slam_toolbox
can look up transforms at scan timestamps.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GzOdomBridge(Node):
    def __init__(self):
        super().__init__("gz_odom_bridge")
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.get_logger().info("gz_odom_bridge ready — waiting for /odom")

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GzOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
