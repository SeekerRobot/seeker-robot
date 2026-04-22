#!/usr/bin/env python3
"""YOLO-driven object seeker with a tri-mode state machine.

Modes
-----
WANDER        — frontier/coverage exploration with no target; the default
                at startup and the mode resumed after a cancel or reached.
SEEK          — activated by a `SeekObject` action goal. Explores while
                watching `/vision/detections` for the requested COCO class
                and approaches when the bbox is large enough.
PERFORM_MOVE  — transient; triggered by the `PerformMove` service. Emits a
                `HexapodCmd(MODE_DANCE)` plus a spin `cmd_vel` for sim
                visibility, then restores the prior mode.

Sub-state within WANDER / SEEK:
  WAITING_FOR_NAV2 -> INITIAL_ROTATION -> FRONTIER_NAV -> COVERAGE_NAV
                                                       +-> APPROACH_OBJECT
                                                       +-> OBJECT_REACHED

Topics / interfaces
-------------------
  Subscribed:  /vision/detections  (mcu_msgs/DetectedObjectArray)
               /map                (nav_msgs/OccupancyGrid)
  Published:   /cmd_vel            (geometry_msgs/Twist) [rotate / spin]
               /mcu/hexapod_cmd    (mcu_msgs/HexapodCmd) [dance]
  Action:      /navigate_to_pose   (nav2_msgs/NavigateToPose) — client
  Action:      /seek_object        (mcu_msgs/action/SeekObject) — server
  Service:     /perform_move       (mcu_msgs/srv/PerformMove) — server
  Service:     /wander             (std_srvs/Trigger) — cancel seek, resume exploration
"""

import enum
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Twist, Quaternion, Point
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from std_srvs.srv import Trigger
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import ClearEntireCostmap
from mcu_msgs.msg import DetectedObjectArray, HexapodCmd
from mcu_msgs.srv import PerformMove
from mcu_msgs.action import SeekObject

from .search_voice import SearchVoice


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

ROTATION_SPEED = 0.2            # rad/s — slow enough for SLAM scan matching to keep up

MIN_BBOX_AREA      = 1000.0     # px² — min bbox area to trigger approach
REACHED_BBOX_AREA  = 18000.0    # px² — bbox this big = close enough
MIN_CONFIDENCE     = 0.20       # YOLO confidence threshold

APPROACH_DISTANCE = 0.5         # metres per approach step
CAMERA_HFOV       = 1.396       # rad — matches seeker_hexapod.urdf.xacro

MIN_FRONTIER_SIZE     = 3
VISITED_RADIUS        = 0.3
MAX_VISITED_FRONTIERS = 100

COVERAGE_SPACING    = 1.0
COVERAGE_MARGIN     = 0.35
NEAR_UNKNOWN_MARGIN = 1.5

DANCE_SPIN_RATE = 1.0           # rad/s during a PERFORM_MOVE dance


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class Mode(enum.IntEnum):
    WANDER = 0
    SEEK = 1
    PERFORM_MOVE = 2


class SubState(enum.IntEnum):
    WAITING_FOR_NAV2 = 0
    INITIAL_ROTATION = 1
    FRONTIER_NAV     = 2
    COVERAGE_NAV     = 3
    APPROACH_OBJECT  = 4
    OBJECT_REACHED   = 5
    SCAN_ROTATION    = 6  # in-place 360° scan between SEEK waypoints


class ObjectSeeker(Node):
    def __init__(self):
        super().__init__("object_seeker")

        cb_group = ReentrantCallbackGroup()

        # cmd_vel caps (belt-and-braces; MCU also clamps). Defaults match the
        # hexapod gait controller's safe envelope. Override via ros-args.
        self.declare_parameter("max_linear_x", 0.1)
        self.declare_parameter("max_linear_y", 0.05)
        self.declare_parameter("max_angular_z_rad", 0.524)  # ~30 deg/s
        self._max_lin_x = float(self.get_parameter("max_linear_x").value)
        self._max_lin_y = float(self.get_parameter("max_linear_y").value)
        self._max_ang_z = float(self.get_parameter("max_angular_z_rad").value)

        # -- Publishers / subscribers --
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._hex_pub = self.create_publisher(HexapodCmd, "/mcu/hexapod_cmd", 10)
        self._tts_pub = self.create_publisher(String, "/audio_tts_input", 10)
        self._file_pub = self.create_publisher(String, "/audio_play_file", 10)

        try:
            sounds_dir = os.path.join(
                get_package_share_directory("seeker_tts"), "sounds"
            )
        except PackageNotFoundError:
            sounds_dir = ""
            self.get_logger().warn(
                "seeker_tts package not found; static WAVs will fall back to dynamic TTS"
            )
        self._voice = SearchVoice(self, self._tts_pub, self._file_pub, sounds_dir)
        self._last_announced_substate: SubState | None = None

        self.create_subscription(
            DetectedObjectArray, "/vision/detections",
            self._on_detections, 10, callback_group=cb_group,
        )

        # slam_toolbox uses TRANSIENT_LOCAL durability on /map.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos, callback_group=cb_group,
        )

        # -- Nav2 action client --
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=cb_group,
        )
        self._current_goal_handle = None
        self._navigating = False

        # -- SLAM / Nav2 reset clients (used at start of each SEEK) --
        # Cycling slam_toolbox's lifecycle wipes the occupancy grid; clearing the
        # Nav2 costmaps drops any stale obstacles so the robot is forced to
        # re-explore for each new target.
        self._slam_lifecycle_client = self.create_client(
            ChangeState, "/slam_toolbox/change_state", callback_group=cb_group,
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=cb_group,
        )
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=cb_group,
        )

        # -- TF --
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # -- Mode + sub-state --
        self._mode: Mode = Mode.WANDER
        self._substate: SubState = SubState.WAITING_FOR_NAV2
        self._prior_mode: Mode = Mode.WANDER
        self._prior_substate: SubState = SubState.FRONTIER_NAV
        self._target_class: str = ""

        self._latest_map = None
        self._visited_frontiers: deque[tuple[float, float]] = deque(maxlen=MAX_VISITED_FRONTIERS)

        self._last_yaw = None
        self._rotation_accumulated = 0.0
        # State to return to after a SCAN_ROTATION completes
        self._scan_return_substate: SubState = SubState.FRONTIER_NAV

        self._frontier_retries = 0
        self._max_frontier_retries = 10
        self._frontier_retry_ticks = 0
        self._frontier_retry_interval = 10

        self._coverage_waypoints: list[tuple[float, float]] = []
        self._coverage_index = 0

        # Detection tracking (target class only)
        self._target_bearing = 0.0
        self._target_bbox_area = 0.0
        self._approach_attempts = 0
        self._max_approach_attempts = 15

        # Action / service bookkeeping
        self._active_goal_handle = None
        self._nav2_goal_count = 0
        self._final_position = Point()
        self._dance_spinning = False

        # -- Action & service servers --
        self._seek_server = ActionServer(
            self, SeekObject, "seek_object",
            execute_callback=self._seek_execute,
            goal_callback=self._seek_goal_accept,
            cancel_callback=self._seek_cancel_accept,
            callback_group=cb_group,
        )
        self._perform_srv = self.create_service(
            PerformMove, "perform_move", self._perform_move_cb,
            callback_group=cb_group,
        )
        self._wander_srv = self.create_service(
            Trigger, "wander", self._wander_cb,
            callback_group=cb_group,
        )

        # 10 Hz tick
        self._dt = 0.1
        self._timer = self.create_timer(self._dt, self._tick, callback_group=cb_group)

        self.get_logger().info(
            "ObjectSeeker started — mode=WANDER, waiting for Nav2 action server"
        )

    # ------------------------------------------------------------------
    # Main tick (10 Hz) — advances WANDER/SEEK sub-state machine.
    # PERFORM_MOVE is driven entirely by the service callback.
    # ------------------------------------------------------------------

    def _tick(self):
        if self._mode == Mode.PERFORM_MOVE:
            if self._dance_spinning:
                self._publish_cmd(0.0, DANCE_SPIN_RATE)
            return

        if self._mode == Mode.SEEK and self._substate in (
            SubState.FRONTIER_NAV, SubState.COVERAGE_NAV, SubState.SCAN_ROTATION,
        ):
            now_ns = self.get_clock().now().nanoseconds
            if self._substate != self._last_announced_substate:
                self._voice.reset_periodic_timer(now_ns)
                self._last_announced_substate = self._substate
            self._voice.maybe_periodic(now_ns, thing=self._target_class)

        if self._substate == SubState.WAITING_FOR_NAV2:
            if self._nav_client.server_is_ready():
                self.get_logger().info(
                    "Nav2 action server ready — starting initial rotation"
                )
                self._substate = SubState.INITIAL_ROTATION
                self._last_yaw = self._get_yaw("odom")
                self._rotation_accumulated = 0.0

        elif self._substate == SubState.INITIAL_ROTATION:
            self._publish_cmd(linear=0.0, angular=ROTATION_SPEED)

            current_yaw = self._get_yaw("odom")
            if self._last_yaw is not None and current_yaw is not None:
                delta = self._angle_diff(current_yaw, self._last_yaw)
                self._rotation_accumulated += abs(delta)
            self._last_yaw = current_yaw

            if self._rotation_accumulated >= 2.0 * math.pi:
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    "Initial 360° rotation complete — starting frontier exploration"
                )
                self._substate = SubState.FRONTIER_NAV
                self._send_next_frontier_goal()

        elif self._substate == SubState.FRONTIER_NAV:
            if not self._navigating and self._frontier_retries > 0:
                self._frontier_retry_ticks += 1
                if self._frontier_retry_ticks >= self._frontier_retry_interval:
                    self._frontier_retry_ticks = 0
                    self._send_next_frontier_goal()

        elif self._substate == SubState.SCAN_ROTATION:
            self._publish_cmd(linear=0.0, angular=ROTATION_SPEED)

            current_yaw = self._get_yaw("odom")
            if self._last_yaw is not None and current_yaw is not None:
                delta = self._angle_diff(current_yaw, self._last_yaw)
                self._rotation_accumulated += abs(delta)
            self._last_yaw = current_yaw

            if self._rotation_accumulated >= 2.0 * math.pi:
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    f"Scan-rotation complete — resuming {self._scan_return_substate.name}"
                )
                self._substate = self._scan_return_substate
                if self._substate == SubState.FRONTIER_NAV:
                    self._send_next_frontier_goal()
                elif self._substate == SubState.COVERAGE_NAV:
                    self._send_next_coverage_goal()

        # COVERAGE_NAV, APPROACH_OBJECT, OBJECT_REACHED: goal-driven, no tick work

    # ------------------------------------------------------------------
    # Detections callback — filters by current target_class when in SEEK
    # ------------------------------------------------------------------

    def _on_detections(self, msg: DetectedObjectArray):
        if self._mode != Mode.SEEK:
            # self.get_logger().info(f"Ignoring detection: mode={self._mode.name}", throttle_duration_sec=2.0)
            return
        if not self._target_class:
            return
            
        if self._substate in (SubState.WAITING_FOR_NAV2, SubState.INITIAL_ROTATION,
                              SubState.OBJECT_REACHED):
            return

        best = None
        for d in msg.detections:
            # self.get_logger().info(f"Seen: {d.class_name} (conf={d.confidence:.2f})")
            if d.class_name != self._target_class:
                continue
            if d.confidence < MIN_CONFIDENCE:
                self.get_logger().info(f"Target '{d.class_name}' conf too low: {d.confidence:.2f} < {MIN_CONFIDENCE}", throttle_duration_sec=1.0)
                continue
            if best is None or d.confidence > best.confidence:
                best = d

        if best is None:
            return

        area = float(best.bbox_w * best.bbox_h)
        self.get_logger().info(f"MATCH: '{self._target_class}' area={area:.0f} (min={MIN_BBOX_AREA})", throttle_duration_sec=0.5)
        offset = (best.bbox_cx - best.image_width / 2.0) / best.image_width
        bearing = -offset * CAMERA_HFOV

        self._target_bearing = bearing
        self._target_bbox_area = area

        if area >= REACHED_BBOX_AREA:
            self._on_object_reached(area)
            return

        if area >= MIN_BBOX_AREA and self._substate != SubState.APPROACH_OBJECT:
            self._start_approach()

    def _start_approach(self):
        self._voice.say_event("object_spotted", thing=self._target_class)
        self._prior_substate = self._substate
        self._substate = SubState.APPROACH_OBJECT
        self._approach_attempts = 0

        if self._current_goal_handle is not None:
            self.get_logger().info(
                f"Cancelling current Nav2 goal — '{self._target_class}' spotted"
            )
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self._navigating = False

        self._send_approach_goal()

    def _send_approach_goal(self):
        if self._approach_attempts >= self._max_approach_attempts:
            self.get_logger().warning(
                "Max approach attempts reached — target may be unreachable"
            )
            self._substate = self._prior_substate or SubState.COVERAGE_NAV
            self._on_goal_finished()
            return

        self._approach_attempts += 1

        robot_x, robot_y = self._get_robot_position()
        robot_yaw = self._get_yaw("map", default=0.0)
        target_yaw = robot_yaw + self._target_bearing
        goal_x = robot_x + APPROACH_DISTANCE * math.cos(target_yaw)
        goal_y = robot_y + APPROACH_DISTANCE * math.sin(target_yaw)

        self.get_logger().info(
            f"Approaching '{self._target_class}' "
            f"(attempt {self._approach_attempts}/{self._max_approach_attempts}): "
            f"bearing={math.degrees(self._target_bearing):.1f}°, "
            f"bbox_area={self._target_bbox_area:.0f}, "
            f"goal=({goal_x:.2f}, {goal_y:.2f})"
        )

        self._navigating = True
        self._send_nav_goal(goal_x, goal_y, "approach")

    def _on_object_reached(self, bbox_area: float):
        self._substate = SubState.OBJECT_REACHED

        if self._current_goal_handle is not None:
            self.get_logger().info("Cancelling Nav2 goal — object reached!")
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

        self._navigating = False
        self._publish_cmd(0.0, 0.0)

        robot_x, robot_y = self._get_robot_position()
        self._final_position.x = robot_x
        self._final_position.y = robot_y
        self._final_position.z = 0.0

        self.get_logger().info(
            f"OBJECT REACHED! '{self._target_class}' bbox area={bbox_area:.0f} px²"
        )
        self._voice.say_event("object_reached", thing=self._target_class)

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        if self._latest_map is None:
            self.get_logger().info(
                f"First map received: {msg.info.width}×{msg.info.height} "
                f"at {msg.info.resolution} m/cell"
            )
        self._latest_map = msg

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------

    def _find_frontiers(self):
        if self._latest_map is None:
            return None

        grid = self._latest_map
        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((height, width))

        free_mask = data == 0
        unknown_mask = data == -1

        padded = np.pad(unknown_mask, 1, constant_values=False)
        unknown_neighbor = (
            padded[:-2, 1:-1] | padded[2:, 1:-1]
            | padded[1:-1, :-2] | padded[1:-1, 2:]
        )
        frontier_mask = free_mask & unknown_neighbor

        if int(np.sum(frontier_mask)) == 0:
            return None

        clusters = self._cluster_frontiers(frontier_mask)
        if not clusters:
            return None

        robot_x, robot_y = self._get_robot_position()
        candidates = []
        for cells in clusters:
            if len(cells) < MIN_FRONTIER_SIZE:
                continue
            centroid_row = sum(c[0] for c in cells) / len(cells)
            centroid_col = sum(c[1] for c in cells) / len(cells)
            map_x = origin_x + centroid_col * resolution
            map_y = origin_y + centroid_row * resolution
            dist = math.hypot(map_x - robot_x, map_y - robot_y)
            candidates.append((map_x, map_y, len(cells) / max(dist, 0.1)))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[2], reverse=True)
        for fx, fy, _score in candidates:
            if all(math.hypot(fx - vx, fy - vy) >= VISITED_RADIUS
                   for vx, vy in self._visited_frontiers):
                return (fx, fy)
        return None

    @staticmethod
    def _dilate_mask(mask: np.ndarray, cells: int) -> np.ndarray:
        result = mask.copy()
        for _ in range(cells):
            padded = np.pad(result, 1, constant_values=False)
            result = (
                result | padded[:-2, 1:-1] | padded[2:, 1:-1]
                | padded[1:-1, :-2] | padded[1:-1, 2:]
            )
        return result

    @staticmethod
    def _cluster_frontiers(frontier_mask):
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
                            if (0 <= nr < height and 0 <= nc < width
                                    and frontier_mask[nr, nc] and not visited[nr, nc]):
                                visited[nr, nc] = True
                                queue.append((nr, nc))
                    clusters.append(cluster)
        return clusters

    # ------------------------------------------------------------------
    # Coverage waypoints
    # ------------------------------------------------------------------

    def _generate_coverage_waypoints(self):
        if self._latest_map is None:
            return []

        grid = self._latest_map
        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        data = np.array(grid.data, dtype=np.int8).reshape((height, width))
        free_mask = data == 0
        occupied_mask = data > 0
        margin_cells = max(1, int(COVERAGE_MARGIN / resolution))
        safe_mask = free_mask & ~self._dilate_mask(occupied_mask, margin_cells)
        unknown_mask = data == -1
        proximity_cells = max(1, int(NEAR_UNKNOWN_MARGIN / resolution))
        near_unknown = safe_mask & self._dilate_mask(unknown_mask, proximity_cells)

        spacing_cells = max(1, int(COVERAGE_SPACING / resolution))
        robot_x, robot_y = self._get_robot_position()

        waypoints = []
        for r in range(spacing_cells, height - spacing_cells, spacing_cells):
            for c in range(spacing_cells, width - spacing_cells, spacing_cells):
                if safe_mask[r, c]:
                    wx = origin_x + c * resolution
                    wy = origin_y + r * resolution
                    priority = 0 if near_unknown[r, c] else 1
                    dist = math.hypot(wx - robot_x, wy - robot_y)
                    waypoints.append((wx, wy, priority, dist))

        waypoints.sort(key=lambda p: (p[2], p[3]))
        waypoints = [(w[0], w[1]) for w in waypoints]
        self.get_logger().info(
            f"Generated {len(waypoints)} coverage waypoints "
            f"(spacing={COVERAGE_SPACING} m, margin={COVERAGE_MARGIN} m)"
        )
        return waypoints

    # ------------------------------------------------------------------
    # Nav2 goal management
    # ------------------------------------------------------------------

    def _send_next_frontier_goal(self):
        target = self._find_frontiers()
        if target is None:
            self._frontier_retries += 1
            if self._frontier_retries >= self._max_frontier_retries:
                self.get_logger().info(
                    "Frontier retries exhausted — switching to coverage navigation"
                )
                if self._mode == Mode.SEEK:
                    self._voice.say_event("coverage_start")
                self._start_coverage_nav()
            return

        self._frontier_retries = 0
        self._frontier_retry_ticks = 0
        self._navigating = True
        goal_x, goal_y = target
        self._visited_frontiers.append((goal_x, goal_y))
        self._send_nav_goal(goal_x, goal_y, "frontier")

    def _start_coverage_nav(self):
        self._substate = SubState.COVERAGE_NAV
        self._coverage_waypoints = self._generate_coverage_waypoints()
        self._coverage_index = 0
        self._send_next_coverage_goal()

    def _send_next_coverage_goal(self):
        if self._coverage_index >= len(self._coverage_waypoints):
            self.get_logger().info(
                "All coverage waypoints visited — rechecking for frontiers"
            )
            self._substate = SubState.FRONTIER_NAV
            self._frontier_retries = 0
            self._frontier_retry_ticks = 0
            self._visited_frontiers.clear()
            self._send_next_frontier_goal()
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
        robot_x, robot_y = self._get_robot_position()
        yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        self._nav2_goal_count += 1
        self.get_logger().info(
            f"Sending Nav2 {label} goal #{self._nav2_goal_count}: "
            f"({goal_x:.2f}, {goal_y:.2f})"
        )

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 goal rejected — trying next")
            self._navigating = False
            self._on_goal_finished()
            return
        self._current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._current_goal_handle = None
        self._navigating = False

        if self._substate == SubState.OBJECT_REACHED:
            return

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached Nav2 goal — finding next target")
        elif status == GoalStatus.STATUS_CANCELED:
            return
        else:
            self.get_logger().warning(
                f"Nav2 goal ended with status {status} — trying next target"
            )
        self._on_goal_finished()

    def _on_goal_finished(self):
        # In SEEK mode, scan 360° between waypoints so YOLO gets every angle.
        # In WANDER mode, skip the scan and chain waypoints straight through.
        if (self._mode == Mode.SEEK
                and self._substate in (SubState.FRONTIER_NAV, SubState.COVERAGE_NAV)):
            self._start_scan_rotation(return_to=self._substate)
            return

        if self._substate == SubState.FRONTIER_NAV:
            self._send_next_frontier_goal()
        elif self._substate == SubState.COVERAGE_NAV:
            self._send_next_coverage_goal()
        elif self._substate == SubState.APPROACH_OBJECT:
            self._send_approach_goal()

    def _start_scan_rotation(self, return_to: SubState):
        self._scan_return_substate = return_to
        self._substate = SubState.SCAN_ROTATION
        self._last_yaw = self._get_yaw("odom")
        self._rotation_accumulated = 0.0

    # ------------------------------------------------------------------
    # Map / costmap reset — wipes SLAM occupancy + Nav2 costmaps so each
    # new SEEK goal starts from a blank slate and is forced to re-explore.
    # ------------------------------------------------------------------

    def _reset_slam_map(self, per_step_timeout: float = 10.0) -> bool:
        # Cycle slam_toolbox through deactivate → cleanup → configure → activate.
        # cleanup is what actually drops the occupancy grid; we re-activate so
        # scans start accumulating into a fresh map immediately.
        steps = (
            ("deactivate", Transition.TRANSITION_DEACTIVATE),
            ("cleanup",    Transition.TRANSITION_CLEANUP),
            ("configure",  Transition.TRANSITION_CONFIGURE),
            ("activate",   Transition.TRANSITION_ACTIVATE),
        )

        if not self._slam_lifecycle_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                "SLAM lifecycle service unavailable — skipping map reset"
            )
            return False

        for name, transition_id in steps:
            req = ChangeState.Request()
            req.transition.id = transition_id
            future = self._slam_lifecycle_client.call_async(req)

            deadline = time.monotonic() + per_step_timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)

            if not future.done():
                self.get_logger().warning(
                    f"SLAM lifecycle '{name}' timed out — map reset aborted"
                )
                return False

            result = future.result()
            if result is None or not result.success:
                self.get_logger().warning(
                    f"SLAM lifecycle '{name}' failed — map reset aborted"
                )
                return False

        self._latest_map = None
        self.get_logger().info("SLAM map reset complete — waiting for map→odom TF")

        # Wait until SLAM is actually publishing map→odom before returning,
        # so Nav2 and the frontier search don't start against a broken TF tree.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                self._tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
                self.get_logger().info("map→odom TF confirmed — SLAM is live")
                return True
            except Exception:
                time.sleep(0.2)

        self.get_logger().warning("SLAM map reset: map→odom TF not seen after 10 s")
        return False

    def _clear_nav2_costmaps(self, timeout: float = 1.0) -> None:
        for label, client in (
            ("global", self._clear_global_costmap_client),
            ("local",  self._clear_local_costmap_client),
        ):
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().warning(
                    f"{label} costmap clear service unavailable — skipping"
                )
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                self.get_logger().warning(f"{label} costmap clear timed out")

    # ------------------------------------------------------------------
    # SeekObject action server
    # ------------------------------------------------------------------

    def _seek_goal_accept(self, goal_request):
        if not goal_request.class_name:
            self.get_logger().warning("SeekObject: empty class_name — rejecting")
            return GoalResponse.REJECT
        if self._active_goal_handle is not None:
            self.get_logger().warning(
                "SeekObject: rejecting new goal — another seek is already active"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _seek_cancel_accept(self, goal_handle):
        return CancelResponse.ACCEPT

    def _seek_execute(self, goal_handle):
        request = goal_handle.request
        timeout_sec = float(request.timeout_sec)
        self.get_logger().info(
            f"SeekObject: seeking '{request.class_name}' "
            f"(timeout={timeout_sec if timeout_sec > 0 else 'none'} s)"
        )

        self._active_goal_handle = goal_handle
        self._target_class = request.class_name
        self._target_bearing = 0.0
        self._target_bbox_area = 0.0
        self._approach_attempts = 0
        self._mode = Mode.SEEK
        self._last_announced_substate = None
        self._voice.say_event("search_start", thing=self._target_class)

        # Fresh seek → drop any mid-exploration state and re-scan the full map.
        # Without this, a seek arriving after WANDER finished exploring would
        # inherit exhausted frontiers / a stale coverage queue and barely move.
        if self._substate not in (SubState.WAITING_FOR_NAV2, SubState.INITIAL_ROTATION):
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._current_goal_handle = None
            self._navigating = False
            self._publish_cmd(0.0, 0.0)

            # Wipe SLAM map + Nav2 costmaps so the robot must re-explore.
            self._reset_slam_map()
            self._clear_nav2_costmaps()

            self._visited_frontiers.clear()
            self._coverage_waypoints = []
            self._coverage_index = 0
            self._frontier_retries = 0
            self._frontier_retry_ticks = 0
            # First action: rotate in place so YOLO sees everything around the
            # current pose before committing to a nav goal.
            self._start_scan_rotation(return_to=SubState.FRONTIER_NAV)

        start_time = self.get_clock().now()
        rate_sec = 0.1

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._voice.say_event("search_canceled")
                    goal_handle.canceled()
                    return self._finish_seek(False, "canceled")

                if self._substate == SubState.OBJECT_REACHED:
                    goal_handle.succeed()
                    return self._finish_seek(True, "reached target")

                if timeout_sec > 0:
                    elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
                    if elapsed > timeout_sec:
                        self._voice.say_event(
                            "search_failed", thing=self._target_class
                        )
                        goal_handle.abort()
                        return self._finish_seek(False, "timeout")

                fb = SeekObject.Feedback()
                # Map internal substate to the 0..4 range declared in the action.
                # SCAN_ROTATION is reported as INITIAL_ROTATION since it's the
                # same user-visible behavior (in-place spin).
                if self._substate == SubState.SCAN_ROTATION:
                    fb.state = int(SubState.INITIAL_ROTATION)
                elif self._substate.value <= SubState.APPROACH_OBJECT:
                    fb.state = int(self._substate)
                else:
                    fb.state = int(SubState.APPROACH_OBJECT)
                fb.last_bbox_area = float(self._target_bbox_area)
                fb.last_bearing = float(self._target_bearing)
                fb.nav2_goal_count = self._nav2_goal_count
                goal_handle.publish_feedback(fb)

                time.sleep(rate_sec)
        except Exception as exc:
            self.get_logger().error(f"SeekObject execute failed: {exc}")
            self._voice.say_event("search_failed", thing=self._target_class)
            if not goal_handle.is_cancel_requested:
                goal_handle.abort()
            return self._finish_seek(False, f"exception: {exc}")

        return self._finish_seek(False, "shutdown")

    def _finish_seek(self, success: bool, message: str) -> SeekObject.Result:
        self._active_goal_handle = None
        self._mode = Mode.WANDER
        self._target_class = ""
        # On reach, stay in OBJECT_REACHED; on cancel/timeout, resume wander.
        if self._substate == SubState.APPROACH_OBJECT:
            self._substate = SubState.FRONTIER_NAV
            self._send_next_frontier_goal()

        result = SeekObject.Result()
        result.success = success
        result.final_position = self._final_position if success else self._current_position_point()
        result.message = message
        return result

    def _current_position_point(self) -> Point:
        x, y = self._get_robot_position()
        p = Point()
        p.x, p.y, p.z = x, y, 0.0
        return p

    # ------------------------------------------------------------------
    # PerformMove service
    # ------------------------------------------------------------------

    def _perform_move_cb(self, request: PerformMove.Request, response: PerformMove.Response):
        move = (request.move_name or "").lower()
        duration = request.duration_sec if request.duration_sec > 0.0 else 3.0

        if move != "dance":
            response.success = False
            response.message = f"unknown move '{request.move_name}'"
            self.get_logger().warning(response.message)
            return response

        self.get_logger().info(f"PerformMove: dance for {duration:.1f} s")

        # Cancel nav so the robot stops before dancing
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self._navigating = False

        self._prior_mode = self._mode
        self._prior_substate = self._substate
        self._mode = Mode.PERFORM_MOVE
        self._dance_spinning = True

        cmd = HexapodCmd()
        cmd.mode = HexapodCmd.MODE_DANCE
        self._hex_pub.publish(cmd)

        time.sleep(duration)

        self._dance_spinning = False
        self._publish_cmd(0.0, 0.0)
        cmd.mode = HexapodCmd.MODE_WALK
        self._hex_pub.publish(cmd)

        # Restore prior mode. If it was SEEK but the goal was cancelled in the
        # meantime, fall back to WANDER.
        if self._prior_mode == Mode.SEEK and self._active_goal_handle is None:
            self._mode = Mode.WANDER
        else:
            self._mode = self._prior_mode
        self._substate = self._prior_substate
        if self._substate in (SubState.FRONTIER_NAV, SubState.COVERAGE_NAV) and not self._navigating:
            self._send_next_frontier_goal()

        response.success = True
        response.message = f"dance complete ({duration:.1f} s)"
        return response

    # ------------------------------------------------------------------
    # Wander service — drop back to exploration from any mode
    # ------------------------------------------------------------------

    def _wander_cb(self, _request: Trigger.Request, response: Trigger.Response):
        prev_mode = self._mode

        # Abort any active seek goal
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.abort()
            except Exception:
                pass
            self._active_goal_handle = None

        # Cancel the current Nav2 goal so the robot stops before re-exploring
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self._navigating = False

        self._mode = Mode.WANDER
        self._target_class = ""
        self._target_bbox_area = 0.0
        self._substate = SubState.FRONTIER_NAV
        self._frontier_retries = 0
        self._send_next_frontier_goal()

        response.success = True
        response.message = f"resumed WANDER (was {prev_mode.name})"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return 0.0, 0.0

    def _get_yaw(self, parent_frame: str, default=None):
        try:
            t = self._tf_buffer.lookup_transform(
                parent_frame, "base_footprint", rclpy.time.Time()
            )
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            return default

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q

    def _publish_cmd(self, linear: float, angular: float, linear_y: float = 0.0):
        msg = Twist()
        msg.linear.x = max(-self._max_lin_x, min(self._max_lin_x, linear))
        msg.linear.y = max(-self._max_lin_y, min(self._max_lin_y, linear_y))
        msg.angular.z = max(-self._max_ang_z, min(self._max_ang_z, angular))
        self._cmd_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = ObjectSeeker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
