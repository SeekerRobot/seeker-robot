#!/usr/bin/env python3
"""Nav2-based frontier exploration with ball detection and approach.

The robot first rotates 360 degrees in place to seed the SLAM map and check
whether the red ball is immediately visible.  It then enters a frontier
exploration loop: it reads the occupancy grid published by slam_toolbox,
finds boundaries between known-free and unknown space, and sends Nav2
NavigateToPose goals to explore them.  If no frontiers remain (e.g. the
LiDAR maps the whole room during rotation) the robot falls back to a
coverage grid: it generates waypoints across the known free space and
navigates to each one, scanning with the camera.

When the ball is spotted the robot cancels its current goal and navigates
toward the ball using the pixel centroid to estimate a bearing. It keeps
approaching until the ball fills enough of the frame to be considered
"reached".

State machine:
  WAITING_FOR_NAV2 -> INITIAL_ROTATION -> FRONTIER_NAV -> COVERAGE_NAV
                                                      \-> APPROACH_BALL -> BALL_REACHED

Topics:
  Subscribed:  /camera/image  (sensor_msgs/Image)
               /map           (nav_msgs/OccupancyGrid)
  Published:   /cmd_vel       (geometry_msgs/Twist)   [rotation phase only]
  Action:      /navigate_to_pose  (nav2_msgs/NavigateToPose)
"""

import enum
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

# Rotation phase
ROTATION_SPEED = 0.4          # rad/s during initial 360-degree scan

# Ball detection — HSV thresholds for a red sphere
HSV_RED_LO1 = np.array([0,   120,  70])
HSV_RED_HI1 = np.array([10,  255, 255])
HSV_RED_LO2 = np.array([170, 120,  70])
HSV_RED_HI2 = np.array([180, 255, 255])
MIN_BALL_PIXELS = 2500         # min red pixels to trigger approach
BALL_REACHED_PIXELS = 8000    # ball fills enough of frame = close enough

# Ball approach
APPROACH_DISTANCE = 0.5       # metres to step toward ball each approach
CAMERA_HFOV = 1.047           # horizontal FoV in radians (~60 degrees)

# Frontier detection
MIN_FRONTIER_SIZE = 3         # minimum cells in a cluster to be considered
VISITED_RADIUS = 0.3          # skip frontiers within this distance of a visited one

# Coverage grid
COVERAGE_SPACING = 1.0        # metres between waypoints in coverage grid
COVERAGE_MARGIN = 0.35        # stay this far from obstacles / walls


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class State(enum.Enum):
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    INITIAL_ROTATION = "INITIAL_ROTATION"
    FRONTIER_NAV = "FRONTIER_NAV"
    COVERAGE_NAV = "COVERAGE_NAV"
    APPROACH_BALL = "APPROACH_BALL"
    BALL_REACHED = "BALL_REACHED"


class BallSearcher(Node):
    def __init__(self):
        super().__init__("ball_searcher")

        # -- Publishers / subscribers --
        self._bridge = CvBridge()
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image", self._on_image, 10)

        # slam_toolbox publishes /map with TRANSIENT_LOCAL durability —
        # we must match it or the subscription silently drops messages.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos
        )

        # -- Nav2 action client --
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self._current_goal_handle = None
        self._navigating = False  # True while a Nav2 goal is active

        # -- TF listener (robot pose lookups) --
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # -- State --
        self._state = State.WAITING_FOR_NAV2
        self._previous_state = None  # state before APPROACH_BALL
        self._latest_map = None
        self._visited_frontiers: list[tuple[float, float]] = []

        # Rotation tracking
        self._last_yaw = None
        self._rotation_accumulated = 0.0

        # Frontier retry tracking (tick-based, no extra timers)
        self._frontier_retries = 0
        self._max_frontier_retries = 10
        self._frontier_retry_ticks = 0   # counts ticks between retries
        self._frontier_retry_interval = 10  # ticks (1 second at 10Hz)

        # Coverage waypoints (fallback when no frontiers)
        self._coverage_waypoints: list[tuple[float, float]] = []
        self._coverage_index = 0

        # Ball approach tracking
        self._ball_bearing = 0.0      # latest bearing to ball (radians)
        self._ball_pixel_count = 0    # latest red pixel count
        self._approach_attempts = 0
        self._max_approach_attempts = 15

        # 10 Hz tick
        self._dt = 0.1
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "BallSearcher started — waiting for Nav2 action server"
        )

    # ------------------------------------------------------------------
    # State machine tick (10 Hz)
    # ------------------------------------------------------------------

    def _tick(self):
        if self._state == State.WAITING_FOR_NAV2:
            if self._nav_client.server_is_ready():
                self.get_logger().info(
                    "Nav2 action server ready — starting initial rotation"
                )
                self._state = State.INITIAL_ROTATION
                self._last_yaw = self._get_robot_yaw()
                self._rotation_accumulated = 0.0

        elif self._state == State.INITIAL_ROTATION:
            self._publish_cmd(linear=0.0, angular=ROTATION_SPEED)

            current_yaw = self._get_robot_yaw()
            if self._last_yaw is not None:
                delta = self._angle_diff(current_yaw, self._last_yaw)
                self._rotation_accumulated += abs(delta)
            self._last_yaw = current_yaw

            if self._rotation_accumulated >= 2.0 * math.pi:
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    "Initial 360-degree rotation complete — "
                    "starting frontier exploration"
                )
                self._state = State.FRONTIER_NAV
                self._send_next_frontier_goal()

        elif self._state == State.FRONTIER_NAV:
            # Tick-based retry: if we're waiting to retry (not navigating),
            # count ticks and retry when interval elapsed
            if not self._navigating and self._frontier_retries > 0:
                self._frontier_retry_ticks += 1
                if self._frontier_retry_ticks >= self._frontier_retry_interval:
                    self._frontier_retry_ticks = 0
                    self._send_next_frontier_goal()

        elif self._state == State.COVERAGE_NAV:
            pass  # driven by goal callbacks

        elif self._state == State.APPROACH_BALL:
            pass  # driven by goal callbacks + camera

        elif self._state == State.BALL_REACHED:
            pass  # terminal

    # ------------------------------------------------------------------
    # Camera callback — ball detection via HSV thresholding
    # ------------------------------------------------------------------

    def _on_image(self, msg: Image):
        if self._state == State.BALL_REACHED:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"cv_bridge error: {exc}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, HSV_RED_LO1, HSV_RED_HI1)
        mask2 = cv2.inRange(hsv, HSV_RED_LO2, HSV_RED_HI2)
        mask = cv2.bitwise_or(mask1, mask2)

        red_pixels = int(np.sum(mask > 0))

        if red_pixels > 50:
            self.get_logger().info(f"Red pixels detected: {red_pixels}")

        if red_pixels < MIN_BALL_PIXELS:
            # If we were approaching but lost sight, keep going to last goal
            return

        # Compute bearing to ball from pixel centroid
        img_width = frame.shape[1]
        _, xs = np.where(mask > 0)
        centroid_x = float(np.mean(xs))
        # Normalized offset from image center: -0.5 (left) to +0.5 (right)
        offset = (centroid_x - img_width / 2.0) / img_width
        # Convert to bearing angle (negative = ball is to the left)
        bearing = -offset * CAMERA_HFOV

        self._ball_bearing = bearing
        self._ball_pixel_count = red_pixels

        # Ball is close enough — we've reached it
        if red_pixels >= BALL_REACHED_PIXELS:
            self._on_ball_reached(red_pixels)
            return

        # Ball spotted but not close enough — start approaching
        if self._state != State.APPROACH_BALL:
            self._start_approach()

    def _start_approach(self):
        """Cancel current goal and switch to APPROACH_BALL state."""
        self._previous_state = self._state
        self._state = State.APPROACH_BALL
        self._approach_attempts = 0

        # Cancel any active Nav2 goal
        if self._current_goal_handle is not None:
            self.get_logger().info("Cancelling current goal — ball spotted!")
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self._navigating = False

        self._send_approach_goal()

    def _send_approach_goal(self):
        """Send a Nav2 goal toward the ball based on camera bearing."""
        if self._approach_attempts >= self._max_approach_attempts:
            self.get_logger().warning(
                "Max approach attempts reached — ball may be unreachable"
            )
            # Go back to previous search state
            self._state = self._previous_state or State.COVERAGE_NAV
            self._on_goal_finished()
            return

        self._approach_attempts += 1

        robot_x, robot_y = self._get_robot_position()
        robot_yaw = self._get_robot_yaw()
        if robot_yaw is None:
            robot_yaw = 0.0

        # Compute world-frame angle toward ball
        ball_yaw = robot_yaw + self._ball_bearing

        # Step toward the ball
        goal_x = robot_x + APPROACH_DISTANCE * math.cos(ball_yaw)
        goal_y = robot_y + APPROACH_DISTANCE * math.sin(ball_yaw)

        self.get_logger().info(
            f"Approaching ball (attempt {self._approach_attempts}/"
            f"{self._max_approach_attempts}): bearing={math.degrees(self._ball_bearing):.1f}deg, "
            f"pixels={self._ball_pixel_count}, goal=({goal_x:.2f}, {goal_y:.2f})"
        )

        self._navigating = True
        self._send_nav_goal(goal_x, goal_y, "approach")

    def _on_ball_reached(self, pixel_count: int):
        """Ball is close enough — stop everything."""
        self._state = State.BALL_REACHED

        if self._current_goal_handle is not None:
            self.get_logger().info("Cancelling Nav2 goal — ball reached!")
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        self._navigating = False
        self._publish_cmd(0.0, 0.0)
        self._timer.cancel()
        self.get_logger().info(
            f"BALL REACHED! Red pixel area: {pixel_count} px. Stopping."
        )

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        if self._latest_map is None:
            self.get_logger().info(
                f"First map received: {msg.info.width}x{msg.info.height} "
                f"at {msg.info.resolution}m/cell"
            )
        self._latest_map = msg

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------

    def _find_frontiers(self):
        """Analyse the latest occupancy grid and return the best frontier.

        A frontier cell is a free cell (value 0) adjacent to at least one
        unknown cell (value -1).  Frontier cells are clustered via BFS,
        scored by size / distance, and the best centroid is returned as
        (x, y) in map frame, or None if no frontiers remain.
        """
        if self._latest_map is None:
            return None

        grid = self._latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        free_mask = data == 0
        unknown_mask = data == -1

        free_count = int(np.sum(free_mask))
        unknown_count = int(np.sum(unknown_mask))
        occupied_count = int(np.sum(data > 0))
        self.get_logger().info(
            f"Map {width}x{height}: free={free_count}, "
            f"unknown={unknown_count}, occupied={occupied_count}"
        )

        # Dilate unknown_mask by 1 cell (4-connected) using numpy shifts
        padded = np.pad(unknown_mask, 1, constant_values=False)
        unknown_neighbor = (
            padded[:-2, 1:-1]   # up
            | padded[2:, 1:-1]  # down
            | padded[1:-1, :-2] # left
            | padded[1:-1, 2:]  # right
        )
        frontier_mask = free_mask & unknown_neighbor

        frontier_count = int(np.sum(frontier_mask))
        self.get_logger().info(f"Frontier cells: {frontier_count}")

        if frontier_count == 0:
            return None

        # BFS connected-component labelling
        clusters = self._cluster_frontiers(frontier_mask)

        self.get_logger().info(
            f"Frontier clusters: {len(clusters)}, "
            f"sizes: {sorted([len(c) for c in clusters], reverse=True)[:5]}"
        )

        if not clusters:
            return None

        robot_x, robot_y = self._get_robot_position()

        candidates = []
        for cells in clusters:
            if len(cells) < MIN_FRONTIER_SIZE:
                continue

            # Centroid in grid coords -> map coords
            rows = [c[0] for c in cells]
            cols = [c[1] for c in cells]
            centroid_row = sum(rows) / len(rows)
            centroid_col = sum(cols) / len(cols)

            map_x = origin_x + centroid_col * resolution
            map_y = origin_y + centroid_row * resolution

            dist = math.hypot(map_x - robot_x, map_y - robot_y)
            score = len(cells) / max(dist, 0.1)

            candidates.append((map_x, map_y, score))

        if not candidates:
            return None

        # Sort by score descending
        candidates.sort(key=lambda c: c[2], reverse=True)

        # Skip centroids too close to previously visited frontiers
        for fx, fy, _score in candidates:
            too_close = False
            for vx, vy in self._visited_frontiers:
                if math.hypot(fx - vx, fy - vy) < VISITED_RADIUS:
                    too_close = True
                    break
            if not too_close:
                return (fx, fy)

        return None

    @staticmethod
    def _cluster_frontiers(frontier_mask):
        """BFS connected-component labelling on a boolean mask."""
        height, width = frontier_mask.shape
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        for r in range(height):
            for c in range(width):
                if frontier_mask[r, c] and not visited[r, c]:
                    cluster = []
                    queue = deque([(r, c)])
                    visited[r, c] = True
                    while queue:
                        cr, cc = queue.popleft()
                        cluster.append((cr, cc))
                        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                            nr, nc = cr + dr, cc + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and frontier_mask[nr, nc]
                                and not visited[nr, nc]
                            ):
                                visited[nr, nc] = True
                                queue.append((nr, nc))
                    clusters.append(cluster)

        return clusters

    # ------------------------------------------------------------------
    # Coverage grid generation (fallback when no frontiers)
    # ------------------------------------------------------------------

    def _generate_coverage_waypoints(self):
        """Generate a grid of waypoints across known free space.

        Uses the occupancy grid to find free cells, then places waypoints
        on a regular grid (COVERAGE_SPACING apart).  Only keeps waypoints
        that are in free space and far enough from obstacles.  Sorts by
        distance from the robot so the nearest waypoint is visited first.
        """
        if self._latest_map is None:
            return []

        grid = self._latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        # Build a mask of cells that are safe to navigate to:
        # free (0) AND not too close to occupied cells
        free_mask = data == 0
        occupied_mask = data > 0

        # Dilate occupied cells by COVERAGE_MARGIN / resolution pixels
        # to create a buffer zone around obstacles
        margin_cells = max(1, int(COVERAGE_MARGIN / resolution))
        dilated_occupied = occupied_mask.copy()
        for _ in range(margin_cells):
            padded = np.pad(dilated_occupied, 1, constant_values=False)
            dilated_occupied = (
                dilated_occupied
                | padded[:-2, 1:-1]
                | padded[2:, 1:-1]
                | padded[1:-1, :-2]
                | padded[1:-1, 2:]
            )

        safe_mask = free_mask & ~dilated_occupied

        # Build a "near unknown" mask: free cells within a few cells of unknown
        unknown_mask = data == -1
        near_unknown = np.zeros_like(unknown_mask)
        proximity_cells = max(1, int(1.5 / resolution))  # 1.5m from unknown
        dilated_unknown = unknown_mask.copy()
        for _ in range(proximity_cells):
            padded = np.pad(dilated_unknown, 1, constant_values=False)
            dilated_unknown = (
                dilated_unknown
                | padded[:-2, 1:-1]
                | padded[2:, 1:-1]
                | padded[1:-1, :-2]
                | padded[1:-1, 2:]
            )
        near_unknown = safe_mask & dilated_unknown

        # Generate grid waypoints
        spacing_cells = max(1, int(COVERAGE_SPACING / resolution))
        robot_x, robot_y = self._get_robot_position()

        waypoints = []
        for r in range(spacing_cells, height - spacing_cells, spacing_cells):
            for c in range(spacing_cells, width - spacing_cells, spacing_cells):
                if safe_mask[r, c]:
                    wx = origin_x + c * resolution
                    wy = origin_y + r * resolution
                    # Prioritize waypoints near unexplored areas
                    priority = 0 if near_unknown[r, c] else 1
                    dist = math.hypot(wx - robot_x, wy - robot_y)
                    waypoints.append((wx, wy, priority, dist))

        # Sort: near-unknown first, then by distance
        waypoints.sort(key=lambda p: (p[2], p[3]))
        waypoints = [(w[0], w[1]) for w in waypoints]

        self.get_logger().info(
            f"Generated {len(waypoints)} coverage waypoints "
            f"(spacing={COVERAGE_SPACING}m, margin={COVERAGE_MARGIN}m)"
        )
        return waypoints

    # ------------------------------------------------------------------
    # Nav2 goal management
    # ------------------------------------------------------------------

    def _send_next_frontier_goal(self):
        """Find the next frontier and send a NavigateToPose goal."""
        target = self._find_frontiers()

        if target is None:
            self._frontier_retries += 1
            if self._latest_map is None:
                self.get_logger().warning(
                    f"No map data yet — retry {self._frontier_retries}/"
                    f"{self._max_frontier_retries}"
                )
            else:
                self.get_logger().info(
                    f"No frontiers found — retry {self._frontier_retries}/"
                    f"{self._max_frontier_retries}"
                )

            if self._frontier_retries >= self._max_frontier_retries:
                # Frontiers exhausted — switch to coverage grid
                self.get_logger().info(
                    "Frontier retries exhausted — switching to coverage navigation"
                )
                self._start_coverage_nav()
            # else: _tick() will count ticks and call us again
            return

        self._frontier_retries = 0
        self._frontier_retry_ticks = 0
        self._navigating = True

        goal_x, goal_y = target
        self._visited_frontiers.append((goal_x, goal_y))
        self._send_nav_goal(goal_x, goal_y, "frontier")

    def _start_coverage_nav(self):
        """Switch to coverage navigation mode."""
        self._state = State.COVERAGE_NAV
        self._coverage_waypoints = self._generate_coverage_waypoints()
        self._coverage_index = 0
        self._send_next_coverage_goal()

    def _send_next_coverage_goal(self):
        """Send the next coverage waypoint as a Nav2 goal."""
        if self._coverage_index >= len(self._coverage_waypoints):
            self.get_logger().info(
                "All coverage waypoints visited — rechecking for frontiers"
            )
            # Map may have changed during coverage — recheck for frontiers
            self._state = State.FRONTIER_NAV
            self._frontier_retries = 0
            self._frontier_retry_ticks = 0
            self._visited_frontiers.clear()
            target = self._find_frontiers()
            if target is not None:
                self.get_logger().info("New frontiers found after coverage!")
                self._send_next_frontier_goal()
            else:
                self.get_logger().info(
                    "No new frontiers — exploration complete, "
                    "ball not found in mapped area"
                )
                self._publish_cmd(0.0, 0.0)
            return

        wx, wy = self._coverage_waypoints[self._coverage_index]
        self._coverage_index += 1
        self.get_logger().info(
            f"Coverage waypoint {self._coverage_index}/"
            f"{len(self._coverage_waypoints)}: ({wx:.2f}, {wy:.2f})"
        )
        self._navigating = True
        self._send_nav_goal(wx, wy, "coverage")

    def _send_nav_goal(self, goal_x: float, goal_y: float, label: str):
        """Send a NavigateToPose goal to the given map coordinates."""
        robot_x, robot_y = self._get_robot_position()
        yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        self.get_logger().info(
            f"Sending Nav2 {label} goal: ({goal_x:.2f}, {goal_y:.2f})"
        )

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _nav_feedback_cb(self, feedback_msg):
        pass  # could log progress if desired

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 goal rejected — trying next")
            self._navigating = False
            self._on_goal_finished()
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._current_goal_handle = None
        self._navigating = False

        if self._state == State.BALL_REACHED:
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached Nav2 goal — finding next target")
        elif status == GoalStatus.STATUS_CANCELED:
            # Goal was cancelled (e.g. ball spotted during nav) — don't chain
            return
        else:
            self.get_logger().warning(
                f"Nav2 goal ended with status {status} — trying next target"
            )

        self._on_goal_finished()

    def _on_goal_finished(self):
        """Route to the correct next-goal method based on current state."""
        if self._state == State.FRONTIER_NAV:
            self._send_next_frontier_goal()
        elif self._state == State.COVERAGE_NAV:
            self._send_next_coverage_goal()
        elif self._state == State.APPROACH_BALL:
            # Reached the approach waypoint — send another if ball still visible
            self._send_approach_goal()

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_robot_position(self):
        """Get robot (x, y) in the map frame via TF."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
            )
        except Exception:
            return (0.0, 0.0)

    def _get_robot_yaw(self):
        """Get robot yaw in odom frame (for rotation tracking)."""
        try:
            t = self._tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time()
            )
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            return None

    def _get_robot_yaw_map(self):
        """Get robot yaw in the map frame (for approach bearing)."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            return 0.0

    @staticmethod
    def _angle_diff(a, b):
        """Signed shortest-path difference a - b, wrapped to [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    # ------------------------------------------------------------------
    # Movement helpers
    # ------------------------------------------------------------------

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = BallSearcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
