#!/usr/bin/env python3
"""Remap Gazebo model pose TF to odom -> base_footprint for slam_toolbox."""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class GzOdomBridge(Node):
    def __init__(self):
        super().__init__("gz_odom_bridge")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(TFMessage, "/gz_tf", self._on_tf, 10)

    def _on_tf(self, msg: TFMessage):
        for tf in msg.transforms:
            t = TransformStamped()
            t.header.stamp = tf.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform = tf.transform
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(GzOdomBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
