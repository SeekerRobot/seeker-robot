"""cmd_vel_restrict — always pick higher-%-of-max axis, no scaling.

Sits between velocity_smoother and the MCU. On every incoming Twist:

  1. linear.y is zeroed (hexapod strafing disabled).
  2. Compare |linear.x| / max_linear  vs  |angular.z| / max_angular.
     Whichever is the larger fraction of its own cap wins — the other is
     zeroed. No proportional scaling, just axis selection.

Lets the robot decisively commit to "drive forward" or "turn in place" on
each tick rather than executing a weak combined arc. Removes the ±0.18
hunting that DWB produced and avoids the inter-axis blending that
confuses scan matching.

Subscribes: ~input (remap to upstream smoothed /cmd_vel)
Publishes:  ~output (remap to /cmd_vel going to MCU)

Params:
  linear_deadband_mps   (default 0.02)   — below this, vx counts as zero
  angular_deadband_rps  (default 0.05)   — below this, wz counts as zero
  max_linear_mps        (default 0.15)   — reference for vx normalisation
  max_angular_rps       (default 0.20)   — reference for wz normalisation
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRestrict(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_restrict")

        self.declare_parameter("linear_deadband_mps", 0.02)
        self.declare_parameter("angular_deadband_rps", 0.05)
        self.declare_parameter("max_linear_mps", 0.15)
        self.declare_parameter("max_angular_rps", 0.20)
        # Bias toward linear motion: angular only wins if its fraction of max
        # exceeds linear's fraction by at least this much. 0.15 = 15pp.
        self.declare_parameter("angular_bias", 0.15)
        self._lin_db = float(self.get_parameter("linear_deadband_mps").value)
        self._ang_db = float(self.get_parameter("angular_deadband_rps").value)
        self._max_lin = float(self.get_parameter("max_linear_mps").value)
        self._max_ang = float(self.get_parameter("max_angular_rps").value)
        self._ang_bias = float(self.get_parameter("angular_bias").value)

        self._pub = self.create_publisher(Twist, "~/output", 10)
        self.create_subscription(Twist, "~/input", self._on_cmd, 10)

        self.get_logger().info(
            f"cmd_vel_restrict ready  lin_db={self._lin_db} ang_db={self._ang_db} "
            f"max_lin={self._max_lin} max_ang={self._max_ang} — "
            f"y zeroed; simultaneous x+wz → larger normalised axis wins"
        )

    def _on_cmd(self, msg: Twist) -> None:
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = msg.angular.z

        lin_active = abs(out.linear.x) > self._lin_db
        ang_active = abs(out.angular.z) > self._ang_db
        if lin_active and ang_active:
            lin_frac = abs(out.linear.x) / max(self._max_lin, 1e-6)
            ang_frac = abs(out.angular.z) / max(self._max_ang, 1e-6)
            # Angular must be 15 percentage points higher than linear to win.
            # Biases toward linear motion to prevent ±wz hunting caused by
            # DWB/RPP flipping between near-equal-score trajectories on each
            # tick. Small incidental angular commands now pass through as
            # pure linear.
            if ang_frac >= lin_frac + self._ang_bias:
                out.linear.x = 0.0   # rotation decisively wins — turn in place
            else:
                out.angular.z = 0.0  # default: translation wins

        # Hard-clamp the winning axis to its configured max. RPP's geometric
        # angular formula can emit values well above max_angular_rps when the
        # lookahead point is near-perpendicular; the velocity_smoother is
        # supposed to cap this but is unreliable during lifecycle activation.
        if out.linear.x > self._max_lin:
            out.linear.x = self._max_lin
        elif out.linear.x < -self._max_lin:
            out.linear.x = -self._max_lin
        if out.angular.z > self._max_ang:
            out.angular.z = self._max_ang
        elif out.angular.z < -self._max_ang:
            out.angular.z = -self._max_ang

        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelRestrict()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
