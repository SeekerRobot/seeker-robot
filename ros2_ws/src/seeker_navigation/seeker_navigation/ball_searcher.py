#!/usr/bin/env python3
"""
Hatsune Object Searcher
-----------------------
Autonomously explores a room using Nav2 Frontier Exploration.
Listens to YOLO detection for dynamic object targeting.

States: IDLE -> INITIAL_ROTATION -> FRONTIER_NAV -> APPROACH_OBJECT -> REACHED
"""

import enum
import math
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String, Bool, Float32MultiArray 

# ---------------------------------------------------------------------------
# Tuning Constants
# ---------------------------------------------------------------------------
ROTATION_SPEED = 0.4          # rad/s for the initial 360 scan
OBJECT_REACHED_AREA = 15000   # Stop when object fills this many pixels
APPROACH_DISTANCE = 0.5       # Increment for Nav2 goals during approach
CAMERA_HFOV = 1.396           # ~80 degrees
MIN_FRONTIER_SIZE = 3         # Ignore tiny clusters of unknown space
VISITED_RADIUS = 0.3          # Don't keep going back to the same spot

class State(enum.Enum):
    IDLE = "IDLE"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    INITIAL_ROTATION = "INITIAL_ROTATION"
    FRONTIER_NAV = "FRONTIER_NAV"
    COVERAGE_NAV = "COVERAGE_NAV"
    APPROACH_BALL = "APPROACH_BALL" # Kept name for compatibility
    BALL_REACHED = "BALL_REACHED"   # Kept name for compatibility

class BallSearcher(Node):
    def __init__(self):
        super().__init__("ball_searcher")

        # -- Communication --
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # New Vision Interface
        self.create_subscription(Bool, "/object_found", self._on_object_found, 10)
        self.create_subscription(Float32MultiArray, "/detection_detail", self._on_detection_detail, 10)
        
        # System Triggers
        self.create_subscription(Bool, "/search_trigger", self._on_voice_trigger, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        # -- Nav2 & Transforms --
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # -- State Variables --
        self._state = State.IDLE
        self._latest_map = None
        self._visited_frontiers = deque(maxlen=100)
        self._navigating = False
        self._current_goal_handle = None

        # Tracking targets
        self._target_visible = False
        self._target_area = 0.0
        self._target_bearing = 0.0

        # Exploration helpers
        self._rotation_accumulated = 0.0
        self._last_yaw = None
        self._frontier_retries = 0
        self._max_frontier_retries = 10
        self._coverage_waypoints = []
        self._coverage_index = 0

        self._timer = self.create_timer(0.1, self._tick) # 10Hz
        self.get_logger().info("Hatsune legs (Searcher) initialized. Standing by.")

    # ------------------------------------------------------------------
    # Vision Callbacks (The "Eyes")
    # ------------------------------------------------------------------

    def _on_object_found(self, msg: Bool):
        self._target_visible = msg.data

    def _on_detection_detail(self, msg: Float32MultiArray):
        if len(msg.data) < 3: return
        
        area, cx, frame_width = msg.data
        self._target_area = area

        # Calculate bearing to the target
        offset = (cx - frame_width / 2.0) / frame_width
        self._target_bearing = -offset * CAMERA_HFOV

        # Logic for switching to "Approach" mode
        if self._target_visible and self._state not in (State.BALL_REACHED, State.IDLE):
            if area >= OBJECT_REACHED_AREA:
                self._on_target_reached()
            elif self._state != State.APPROACH_BALL:
                self._start_approach()

    def _start_approach(self):
        self.get_logger().info("🎯 TARGET SPOTTED: Switching to approach mode.")
        self._state = State.APPROACH_BALL
        if self._current_goal_handle:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
        self._send_approach_goal()

    def _on_target_reached(self):
        self._state = State.BALL_REACHED
        self.get_logger().info("🏁 OBJECT REACHED: Goal complete.")
        if self._current_goal_handle:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
        self._navigating = False
        self._publish_cmd(0.0, 0.0)

    # ------------------------------------------------------------------
    # State Machine Core
    # ------------------------------------------------------------------

    def _tick(self):
        if self._state == State.WAITING_FOR_NAV2:
            if self._nav_client.server_is_ready():
                self._state = State.INITIAL_ROTATION
                self._last_yaw = self._get_yaw("odom")
                self._rotation_accumulated = 0.0

        elif self._state == State.INITIAL_ROTATION:
            self._publish_cmd(0.0, ROTATION_SPEED)
            curr = self._get_yaw("odom")
            if self._last_yaw is not None:
                self._rotation_accumulated += abs(self._angle_diff(curr, self._last_yaw))
            self._last_yaw = curr

            if self._rotation_accumulated >= 2.0 * math.pi:
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info("Initial scan complete. Moving to explore.")
                self._state = State.FRONTIER_NAV
                self._send_next_frontier_goal()

    def _on_voice_trigger(self, msg: Bool):
        if msg.data is True and self._state == State.IDLE:
            self.get_logger().info("Voice Trigger Heard! Commencing Search...")
            self._state = State.WAITING_FOR_NAV2
    # ------------------------------------------------------------------
    # Nav2 / Exploration Logic
    # ------------------------------------------------------------------

    def _send_next_frontier_goal(self):
        target = self._find_frontiers()
        if target is None:
            self._frontier_retries += 1
            if self._frontier_retries >= self._max_frontier_retries:
                self._start_coverage_nav()
            return

        self._frontier_retries = 0
        self._navigating = True
        self._visited_frontiers.append(target)
        self._send_nav_goal(target[0], target[1], "frontier")

    def _send_approach_goal(self):
        rx, ry = self._get_robot_position()
        ryaw = self._get_yaw("map", default=0.0)
        target_yaw = ryaw + self._target_bearing
        gx = rx + APPROACH_DISTANCE * math.cos(target_yaw)
        gy = ry + APPROACH_DISTANCE * math.sin(target_yaw)
        self._navigating = True
        self._send_nav_goal(gx, gy, "approach")

    def _send_nav_goal(self, gx, gy, label):
        robot_x, robot_y = self._get_robot_position()
        yaw = math.atan2(gy - robot_y, gx - robot_x)

        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = gx
        msg.pose.pose.position.y = gy
        msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        future = self._nav_client.send_goal_async(msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._navigating = False
            self._on_goal_finished()
            return
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._current_goal_handle = None
        self._navigating = False
        if self._state != State.BALL_REACHED:
            self._on_goal_finished()

    def _on_goal_finished(self):
        if self._state == State.FRONTIER_NAV:
            self._send_next_frontier_goal()
        elif self._state == State.APPROACH_BALL:
            self._send_approach_goal()

    # ------------------------------------------------------------------
    # Frontier Math (The heavy lifting)
    # ------------------------------------------------------------------

    def _on_map(self, msg):
        self._latest_map = msg

    def _find_frontiers(self):
        if self._latest_map is None: return None
        grid = self._latest_map
        data = np.array(grid.data, dtype=np.int8).reshape((grid.info.height, grid.info.width))
        
        # Identify free cells next to unknown cells
        free = data == 0
        unknown = data == -1
        padded = np.pad(unknown, 1, constant_values=False)
        unknown_neighbor = (padded[:-2, 1:-1] | padded[2:, 1:-1] | padded[1:-1, :-2] | padded[1:-1, 2:])
        mask = free & unknown_neighbor
        
        clusters = self._cluster_frontiers(mask)
        if not clusters: return None
        
        # Pick the best centroid (largest/closest)
        rx, ry = self._get_robot_position()
        candidates = []
        for cells in clusters:
            if len(cells) < MIN_FRONTIER_SIZE: continue
            rows, cols = zip(*cells)
            mx = grid.info.origin.position.x + (sum(cols)/len(cols)) * grid.info.resolution
            my = grid.info.origin.position.y + (sum(rows)/len(rows)) * grid.info.resolution
            
            too_close = any(math.hypot(mx-vx, my-vy) < VISITED_RADIUS for vx, vy in self._visited_frontiers)
            if not too_close:
                candidates.append((mx, my, len(cells)/max(math.hypot(mx-rx, my-ry), 0.1)))
        
        if not candidates: return None
        candidates.sort(key=lambda x: x[2], reverse=True)
        return (candidates[0][0], candidates[0][1])

    def _cluster_frontiers(self, mask):
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        clusters = []
        for r in range(h):
            for c in range(w):
                if mask[r,c] and not visited[r,c]:
                    cluster, q = [], deque([(r,c)])
                    visited[r,c] = True
                    while q:
                        cr, cc = q.popleft()
                        cluster.append((cr, cc))
                        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1)):
                            nr, nc = cr+dr, cc+dc
                            if 0<=nr<h and 0<=nc<w and mask[nr,nc] and not visited[nr,nc]:
                                visited[nr,nc] = True
                                q.append((nr,nc))
                    clusters.append(cluster)
        return clusters

    # ------------------------------------------------------------------
    # TF & Math Helpers
    # ------------------------------------------------------------------

    def _get_robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except: return (0.0, 0.0)

    def _get_yaw(self, frame, default=None):
        try:
            t = self._tf_buffer.lookup_transform(frame, "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        except: return default

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi: d -= 2.0*math.pi
        while d < -math.pi: d += 2.0*math.pi
        return d

    def _yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w, q.z = math.cos(yaw/2.0), math.sin(yaw/2.0)
        return q

    def _publish_cmd(self, lin, ang):
        msg = Twist()
        msg.linear.x, msg.angular.z = lin, ang
        self._cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = BallSearcher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        try: node._publish_cmd(0.0, 0.0)
        except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()