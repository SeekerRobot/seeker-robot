"""scan_tilt_filter — drop LiDAR rays tilted into floor/ceiling by body rock.

The hexapod tripod gait pitches/rolls the body on every step. Gazebo's IMU
sensor doesn't populate the orientation quaternion, but the EKF already fuses
IMU angular rates (roll_rate, pitch_rate, yaw_rate) and publishes the full 3-D
orientation in the odom→base_link TF chain. This node looks up that TF each
scan, extracts roll/pitch, then replaces rays whose effective elevation exceeds
max_tilt_rad with inf before republishing on /mcu/scan_filtered.

SLAM and Nav2 costmaps subscribe to /mcu/scan_filtered.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from sensor_msgs.msg import LaserScan

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class ScanTiltFilter(Node):
    def __init__(self) -> None:
        super().__init__('scan_tilt_filter')

        self.declare_parameter('max_tilt_rad', 0.06)  # ~3.5 deg; tune per gait amplitude
        self._max_tilt: float = self.get_parameter('max_tilt_rad').value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._scan_sub = self.create_subscription(
            LaserScan, '/mcu/scan', self._scan_cb, _SENSOR_QOS)
        self._pub = self.create_publisher(LaserScan, '/mcu/scan_filtered', 10)

        # LD14P occasionally emits scans one point off from the previous one
        # (timing drift on its continuous rotation). slam_toolbox's Karto
        # caches the first scan's length and spams "LaserRangeScan contains
        # N, expected M" for every mismatch. Lock to the first length here
        # and pad/truncate — SLAM sees a consistent ray count.
        self._locked_len: int | None = None

        self.get_logger().info(
            f'scan_tilt_filter ready  max_tilt={math.degrees(self._max_tilt):.1f} deg')

    # ------------------------------------------------------------------

    def _get_tilt(self) -> tuple[float, float]:
        """Return (roll, pitch) in radians from TF odom→base_link."""
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr, cosr)
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            pitch = math.asin(sinp)
            return roll, pitch
        except tf2_ros.LookupException:
            return 0.0, 0.0
        except tf2_ros.ExtrapolationException:
            return 0.0, 0.0

    def _scan_cb(self, msg: LaserScan) -> None:
        roll, pitch = self._get_tilt()
        threshold = self._max_tilt

        ranges = list(msg.ranges)
        angle = msg.angle_min
        inc = msg.angle_increment
        dropped = 0

        for i in range(len(ranges)):
            # Effective elevation of this ray given body roll/pitch:
            #   ray body-frame direction: [cos(a), sin(a), 0]
            #   world vertical component: -pitch*cos(a) + roll*sin(a)
            elevation = -pitch * math.cos(angle) + roll * math.sin(angle)
            if abs(elevation) > threshold:
                ranges[i] = float('inf')
                dropped += 1
            angle += inc

        # Normalise scan length against the first-seen count so Karto doesn't
        # log "LaserRangeScan contains N, expected M" on every off-by-one.
        if self._locked_len is None:
            self._locked_len = len(ranges)
        if len(ranges) < self._locked_len:
            ranges.extend([float('inf')] * (self._locked_len - len(ranges)))
        elif len(ranges) > self._locked_len:
            ranges = ranges[: self._locked_len]

        if dropped and self.get_clock().now().nanoseconds % 5_000_000_000 < 100_000_000:
            self.get_logger().debug(
                f'tilt filter: roll={math.degrees(roll):.1f}° '
                f'pitch={math.degrees(pitch):.1f}°  dropped {dropped}/{len(ranges)} rays')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        # angle_max / angle_increment must stay consistent with the locked
        # ray count; recompute angle_max so (max - min) / inc == locked_len - 1.
        out.angle_increment = msg.angle_increment
        out.angle_max = msg.angle_min + msg.angle_increment * (self._locked_len - 1)
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        intensities = list(msg.intensities) if msg.intensities else []
        if intensities:
            if len(intensities) < self._locked_len:
                intensities.extend([0.0] * (self._locked_len - len(intensities)))
            elif len(intensities) > self._locked_len:
                intensities = intensities[: self._locked_len]
        out.intensities = intensities

        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanTiltFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
