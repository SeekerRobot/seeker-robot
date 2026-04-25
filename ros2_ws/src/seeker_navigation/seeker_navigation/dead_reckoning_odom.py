"""dead_reckoning_odom — open-loop odometry from /cmd_vel (no IMU).

Integrates the commanded body-frame velocity into an `odom → base_footprint`
TF and a `nav_msgs/Odometry` message on `/odom`. Used when the gyro/EKF path
is disabled (e.g. to isolate whether a SLAM/Nav2 issue is caused by a bad
IMU fusion).

Drifts over time — any leg slippage or physical perturbation is invisible
here. SLAM Toolbox's scan matching is the only thing that corrects it, via
the `map → odom` TF it publishes.

Publishes:
  /odom    (nav_msgs/Odometry)
  TF       odom → base_footprint

Subscribes:
  /cmd_vel (geometry_msgs/Twist)

Params:
  publish_rate_hz  (default 50.0)  — odometry / TF publish rate
  cmd_timeout_s    (default 0.5)   — if no cmd_vel within this window, the
                                     integrated velocity is reset to zero
"""

import math

import rclpy
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class DeadReckoningOdom(Node):
    def __init__(self) -> None:
        super().__init__("dead_reckoning_odom")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._last_cmd_time = self.get_clock().now()
        self._last_tick = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_bcast = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"dead_reckoning_odom ready (publish_rate={rate} Hz, timeout={self._cmd_timeout} s)"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._wz = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt <= 0.0 or dt > 0.5:
            return

        # Stale cmd → stop integrating
        if (now - self._last_cmd_time).nanoseconds * 1e-9 > self._cmd_timeout:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0

        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        self._x += (self._vx * cos_y - self._vy * sin_y) * dt
        self._y += (self._vx * sin_y + self._vy * cos_y) * dt
        self._yaw += self._wz * dt
        # Wrap yaw to [-pi, pi] to keep quaternion well-conditioned
        while self._yaw > math.pi:
            self._yaw -= 2.0 * math.pi
        while self._yaw < -math.pi:
            self._yaw += 2.0 * math.pi

        q = Quaternion()
        q.z = math.sin(self._yaw / 2.0)
        q.w = math.cos(self._yaw / 2.0)

        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        # Loose covariance — SLAM's map→odom correction is the real source of truth.
        for i in (0, 7, 35):
            odom.pose.covariance[i] = 0.1
        for i in (14, 21, 28):
            odom.pose.covariance[i] = 1.0e6  # z, roll, pitch are undetermined
        for i in (0, 7, 35):
            odom.twist.covariance[i] = 0.01
        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation = q
        self._tf_bcast.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadReckoningOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
