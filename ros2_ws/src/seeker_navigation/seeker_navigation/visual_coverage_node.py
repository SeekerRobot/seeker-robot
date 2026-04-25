"""visual_coverage_node — track which cells the camera has actually pointed at.

The frontier explorer in object_seeker pushes the robot toward unmapped LiDAR
space, but the camera FOV is narrow (~80°) and the robot can drive past whole
regions without aiming the camera at them. This node maintains a 2D grid
aligned with SLAM's `/map` that records every cell the camera has *visually*
swept through. Each tick: look up `map → camera_link`, raycast the FOV cone,
mark free cells SEEN, stop on walls.

Cell semantics in `/visual_coverage`:
  -1  unknown — the underlying /map cell is unknown or occupied (we never
      paint coverage onto walls; they aren't traversable view targets)
   0  unseen-free — free in /map, camera has never pointed through it
 100  seen-free   — free in /map, camera has pointed through it at least once

object_seeker reads this to bias frontier scoring toward unseen-free regions
and to orient each Nav2 goal so the camera faces the largest unseen blob.

Service:
  /visual_coverage/clear (std_srvs/Trigger) — wipes the SEEN history. Called
  by object_seeker at the start of each new SEEK goal so the robot re-sweeps
  for the new target class (YOLO only fires for the active class).
"""

import math

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


_VAL_UNSEEN = 0
_VAL_SEEN = 100
_VAL_UNKNOWN = -1


_MAP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class VisualCoverageNode(Node):
    def __init__(self) -> None:
        super().__init__('visual_coverage')

        # camera_link inherits base_link orientation (URDF: rpy="0 0 0"), so
        # its +X axis points along the lens. camera_optical_frame uses the
        # ROS optical convention (+Z out of lens) which would put yaw 90°
        # off — don't switch to it without remapping the bearing.
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fov_rad', 1.396)
        self.declare_parameter('range_m', 4.0)
        self.declare_parameter('update_hz', 10.0)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('ray_angle_step_rad', 0.035)

        self._camera_frame: str = self.get_parameter('camera_frame').value
        self._map_frame: str = self.get_parameter('map_frame').value
        self._fov_rad: float = float(self.get_parameter('fov_rad').value)
        self._range_m: float = float(self.get_parameter('range_m').value)
        self._ray_step_rad: float = float(self.get_parameter('ray_angle_step_rad').value)

        update_hz = float(self.get_parameter('update_hz').value)
        publish_hz = float(self.get_parameter('publish_hz').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._latest_map: OccupancyGrid | None = None
        self._coverage: np.ndarray | None = None  # int8, shape (H, W)

        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, _MAP_QOS,
        )
        self._pub = self.create_publisher(
            OccupancyGrid, '/visual_coverage', _MAP_QOS,
        )
        self.create_service(
            Trigger, '/visual_coverage/clear', self._clear_cb,
        )

        self.create_timer(1.0 / max(update_hz, 1.0), self._update_tick)
        self.create_timer(1.0 / max(publish_hz, 0.1), self._publish_tick)

        self.get_logger().info(
            f"visual_coverage ready  fov={math.degrees(self._fov_rad):.0f}°  "
            f"range={self._range_m:.1f} m  update={update_hz:.0f} Hz"
        )

    # ------------------------------------------------------------------
    # /map ingestion + coverage grid resize
    # ------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._coverage is None or self._latest_map is None:
            self._coverage = self._fresh_coverage(msg)
            self._latest_map = msg
            return

        old_meta = self._latest_map.info
        new_meta = msg.info
        same_geom = (
            old_meta.width == new_meta.width
            and old_meta.height == new_meta.height
            and abs(old_meta.resolution - new_meta.resolution) < 1e-6
            and abs(old_meta.origin.position.x - new_meta.origin.position.x) < 1e-6
            and abs(old_meta.origin.position.y - new_meta.origin.position.y) < 1e-6
        )

        if same_geom:
            self._latest_map = msg
            return

        # slam_toolbox grew or shifted the map — remap SEEN cells to the new
        # grid by world coords so visual history isn't lost on resize.
        new_coverage = self._fresh_coverage(msg)
        self._blit_seen(self._coverage, old_meta, new_coverage, new_meta)
        self._coverage = new_coverage
        self._latest_map = msg
        self.get_logger().info(
            f"Coverage grid remapped to new map geometry: "
            f"{old_meta.width}x{old_meta.height} → {new_meta.width}x{new_meta.height}"
        )

    @staticmethod
    def _fresh_coverage(msg: OccupancyGrid) -> np.ndarray:
        h, w = msg.info.height, msg.info.width
        return np.full((h, w), _VAL_UNSEEN, dtype=np.int8)

    @staticmethod
    def _blit_seen(
        src: np.ndarray,
        src_meta,
        dst: np.ndarray,
        dst_meta,
    ) -> None:
        """Copy SEEN cells from src into dst, preserving world coordinates."""
        res = src_meta.resolution
        sox = src_meta.origin.position.x
        soy = src_meta.origin.position.y
        dox = dst_meta.origin.position.x
        doy = dst_meta.origin.position.y
        # Cell-coord shift from src to dst.
        dx_cells = int(round((sox - dox) / res))
        dy_cells = int(round((soy - doy) / res))

        sh, sw = src.shape
        dh, dw = dst.shape
        # Source rectangle in dst coords.
        x0 = max(0, dx_cells)
        y0 = max(0, dy_cells)
        x1 = min(dw, dx_cells + sw)
        y1 = min(dh, dy_cells + sh)
        if x1 <= x0 or y1 <= y0:
            return

        # Source slice corresponding to that rectangle.
        sx0 = x0 - dx_cells
        sy0 = y0 - dy_cells
        sx1 = sx0 + (x1 - x0)
        sy1 = sy0 + (y1 - y0)

        src_slice = src[sy0:sy1, sx0:sx1]
        dst_slice = dst[y0:y1, x0:x1]
        seen_mask = src_slice == _VAL_SEEN
        dst_slice[seen_mask] = _VAL_SEEN

    # ------------------------------------------------------------------
    # Raycast tick
    # ------------------------------------------------------------------

    def _update_tick(self) -> None:
        if self._latest_map is None or self._coverage is None:
            return

        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._camera_frame, rclpy.time.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return

        cx = t.transform.translation.x
        cy = t.transform.translation.y
        q = t.transform.rotation
        # Yaw of the camera frame in the map frame. The optical frame's +Z
        # points along the lens, but we're projecting onto the XY plane and
        # what we want is the bearing the lens points along — extracting
        # standard yaw from the quaternion gives that for a level robot.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._raycast(cx, cy, yaw)

    def _raycast(self, cx_world: float, cy_world: float, yaw: float) -> None:
        meta = self._latest_map.info
        res = meta.resolution
        if res <= 0.0:
            return
        ox = meta.origin.position.x
        oy = meta.origin.position.y
        h, w = self._coverage.shape

        cx_cell_f = (cx_world - ox) / res
        cy_cell_f = (cy_world - oy) / res

        n_rays = max(2, int(math.ceil(self._fov_rad / self._ray_step_rad)) + 1)
        max_steps = max(1, int(self._range_m / res))
        half_fov = self._fov_rad * 0.5
        bearings = np.linspace(yaw - half_fov, yaw + half_fov, n_rays)

        map_data = np.array(self._latest_map.data, dtype=np.int8).reshape((h, w))

        for bearing in bearings:
            dx = math.cos(bearing)
            dy = math.sin(bearing)
            for s in range(1, max_steps + 1):
                ix = int(cx_cell_f + dx * s)
                iy = int(cy_cell_f + dy * s)
                if ix < 0 or ix >= w or iy < 0 or iy >= h:
                    break
                map_val = int(map_data[iy, ix])
                if map_val > 0:
                    # Wall — view blocked beyond this cell.
                    break
                if map_val == 0:
                    self._coverage[iy, ix] = _VAL_SEEN
                # map_val == -1 (unknown) — don't mark, but keep walking;
                # the camera physically sees through unmapped space.

    # ------------------------------------------------------------------
    # Publish + clear
    # ------------------------------------------------------------------

    def _publish_tick(self) -> None:
        if self._latest_map is None or self._coverage is None:
            return

        meta = self._latest_map.info
        h, w = self._coverage.shape
        map_data = np.array(self._latest_map.data, dtype=np.int8).reshape((h, w))
        # Composite: SEEN-free where coverage marked, else mirror /map's
        # unknown/wall (-1 / >0 → -1) for clean RViz overlay.
        out = np.where(map_data == 0, self._coverage, _VAL_UNKNOWN).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.info = meta
        msg.data = out.flatten().tolist()
        self._pub.publish(msg)

    def _clear_cb(self, request: Trigger.Request, response: Trigger.Response):
        if self._latest_map is None:
            response.success = False
            response.message = 'no map yet'
            return response
        self._coverage = self._fresh_coverage(self._latest_map)
        self.get_logger().info('Visual coverage cleared by service request')
        response.success = True
        response.message = 'visual coverage cleared'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualCoverageNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
