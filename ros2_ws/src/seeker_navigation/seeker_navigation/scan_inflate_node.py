"""scan_inflate_node — reassemble CompactScan chunks into sensor_msgs/LaserScan.

The MCU splits each LD14P rotation into mcu_msgs/CompactScan chunks on
/mcu/scan_compact (uint16 mm, one IP frame per chunk) to stay under the
2048 B micro-ROS MTU on slow WiFi. This node buffers chunks by scan_id and
republishes a single sensor_msgs/LaserScan on /mcu/scan once all chunks for
a rotation have arrived. 0 mm → inf on the wire; downstream consumers see
the same topic shape they used to get from the MCU directly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from mcu_msgs.msg import CompactScan


_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class ScanInflateNode(Node):
    def __init__(self) -> None:
        super().__init__('scan_inflate')

        self._sub = self.create_subscription(
            CompactScan, '/mcu/scan_compact', self._chunk_cb, _SENSOR_QOS)
        self._pub = self.create_publisher(LaserScan, '/mcu/scan', _SENSOR_QOS)

        self._pending_id: int | None = None
        self._pending: dict[int, CompactScan] = {}

        self.get_logger().info('scan_inflate ready  /mcu/scan_compact → /mcu/scan')

    def _chunk_cb(self, msg: CompactScan) -> None:
        if msg.scan_id != self._pending_id:
            self._pending_id = msg.scan_id
            self._pending = {}
        self._pending[msg.chunk_index] = msg

        if len(self._pending) >= msg.chunk_count:
            chunks = [self._pending.get(i) for i in range(msg.chunk_count)]
            if all(c is not None for c in chunks):
                self._emit(chunks)
            self._pending = {}
            self._pending_id = None

    def _emit(self, chunks: list) -> None:
        first = chunks[0]

        ranges: list[float] = []
        for c in chunks:
            ranges.extend(
                float('inf') if mm == 0 else mm * 0.001 for mm in c.ranges_mm
            )

        out = LaserScan()
        out.header = first.header
        out.angle_min = first.angle_min
        out.angle_increment = first.angle_increment
        # Derive angle_max from the reassembled count so consumers see a
        # self-consistent scan even if a chunk's stored angle_max drifted.
        out.angle_max = (
            first.angle_min + first.angle_increment * max(len(ranges) - 1, 0)
        )
        out.time_increment = first.time_increment
        out.scan_time = first.scan_time
        out.range_min = first.range_min
        out.range_max = first.range_max
        out.ranges = ranges
        out.intensities = []
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanInflateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
