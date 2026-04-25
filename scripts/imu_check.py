"""imu_check.py — visualise IMU orientation as a TF frame next to the robot.

Subscribes to /mcu/imu, broadcasts:
  odom → imu_orientation_test
with translation 1 m to the robot's left of base_link's odom position and
rotation = the IMU's reported orientation quaternion.

In RViz add the `imu_orientation_test` frame to the TF display. With the robot
sitting level you should see an axis-aligned frame floating beside the robot.
When you tilt the robot physically:
  - nose-up (pitch up)         → frame rotates the same way (X axis tips up)
  - left side raised (roll left)  → frame rotates the same way (Y axis tips up)
  - spin CCW (looking from above) → frame rotates CCW about its Z

If the visualised frame tilts the OPPOSITE direction or about the WRONG axis
when you tilt the robot, the BNO085 chip is mounted with a different
orientation than the firmware's setReorientation() compensates for.

Also prints live roll/pitch/yaw at 1 Hz to stdout so you can verify without
opening RViz.

Usage (inside the container):
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_workspaces/install/setup.bash      # so /mcu/imu type is known
  python3 ~/scripts/imu_check.py

Stop with Ctrl+C.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


_IMU_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ImuCheck(Node):
    def __init__(self) -> None:
        super().__init__('imu_check')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_pub = TransformBroadcaster(self)

        self._latest_imu: Imu | None = None
        self._last_print = 0.0

        self.create_subscription(Imu, '/mcu/imu', self._on_imu, _IMU_QOS)
        # 30 Hz broadcast cadence — smooth in RViz, light on CPU.
        self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info(
            "imu_check ready — subscribing /mcu/imu, broadcasting TF "
            "odom→imu_orientation_test (1 m to robot's left of base_link)."
        )

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _tick(self) -> None:
        msg = self._latest_imu
        if msg is None:
            return

        # Anchor position next to base_link in odom so the frame appears beside
        # the robot in RViz. If TF isn't available yet, fall back to (0,0,0.5).
        try:
            base = self._tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
            base_x = base.transform.translation.x
            base_y = base.transform.translation.y
            base_z = base.transform.translation.z
        except Exception:
            base_x = 0.0
            base_y = 0.0
            base_z = 0.0

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'imu_orientation_test'
        # Offset 1 m toward odom +Y (= robot left in REP-103) and 0.3 m up so
        # it sits clearly beside the robot model in RViz.
        out.transform.translation.x = base_x
        out.transform.translation.y = base_y + 1.0
        out.transform.translation.z = base_z + 0.3
        out.transform.rotation = msg.orientation
        self._tf_pub.sendTransform(out)

        # Stdout summary at 1 Hz — easier than tailing topic echo.
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            roll, pitch, yaw = quat_to_rpy(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
            print(
                f"[IMU] roll={math.degrees(roll):+6.1f}°  "
                f"pitch={math.degrees(pitch):+6.1f}°  "
                f"yaw={math.degrees(yaw):+6.1f}°    "
                f"wx={wx:+5.2f}  wy={wy:+5.2f}  wz={wz:+5.2f} rad/s",
                flush=True,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
