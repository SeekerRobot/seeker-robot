#!/usr/bin/env python3
"""Ball searcher node for the Seeker hexapod.

Executes an expanding square-spiral search pattern via /cmd_vel while
using LiDAR for reactive obstacle avoidance and scanning the camera
feed for a red ball. When detected, the robot stops and logs the find.
This is the integration point for later YOLO-based detection.

Topics:
  Subscribed:  /camera/image  (sensor_msgs/Image)
               /lidar/scan    (sensor_msgs/LaserScan)
  Published:   /cmd_vel       (geometry_msgs/Twist)
"""

import enum
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan


# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

# Movement speeds (m/s and rad/s)
LINEAR_SPEED = 0.15   # forward speed during MOVE_FORWARD phase
TURN_SPEED = 0.5      # angular speed during TURN phase (rad/s)
TURN_ANGLE = 1.5708   # 90 degrees in radians

# Spiral parameters
INITIAL_LEG_METERS = 0.5   # length of the first two legs
LEG_INCREMENT = 0.5         # increase leg length every two turns

# Obstacle avoidance — LiDAR-based
OBSTACLE_DIST = 0.4         # meters — stop and turn well before collision
FRONT_FOV_DEG = 60          # degrees — forward cone to check for obstacles

# Ball detection — HSV thresholds for a red sphere
# Red wraps around in HSV: two ranges are needed.
HSV_RED_LO1 = np.array([0,   120,  70])
HSV_RED_HI1 = np.array([10,  255, 255])
HSV_RED_LO2 = np.array([170, 120,  70])
HSV_RED_HI2 = np.array([180, 255, 255])
MIN_BALL_PIXELS = 3000  # minimum red pixel area — triggers at roughly 1.5 m distance


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class State(enum.Enum):
    MOVE_FORWARD = "MOVE_FORWARD"
    TURN = "TURN"
    AVOID_TURN = "AVOID_TURN"    # obstacle avoidance turn
    STOPPED = "STOPPED"


class BallSearcher(Node):
    def __init__(self):
        super().__init__("ball_searcher")

        self._bridge = CvBridge()
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Image, "/camera/image", self._on_image, 10)
        self.create_subscription(LaserScan, "/lidar/scan", self._on_scan, 10)

        # LiDAR state
        self._obstacle_ahead = False

        # Spiral state
        self._state = State.MOVE_FORWARD
        self._leg_length = INITIAL_LEG_METERS
        self._legs_done = 0          # count of completed legs (every 2 → increase length)
        self._elapsed = 0.0          # seconds spent in current state
        self._target_duration = self._leg_length / LINEAR_SPEED

        # Timer drives the movement state machine at 10 Hz
        self._dt = 0.1
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            "BallSearcher started — spiral search with LiDAR obstacle avoidance"
        )

    # ------------------------------------------------------------------
    # LiDAR callback — obstacle detection in front cone
    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan):
        if self._state == State.STOPPED:
            return

        # Determine which scan indices correspond to the front cone.
        # The LiDAR covers 0→2π. "Front" is the region around 0° (and
        # wrapping near 360°).
        half_fov = math.radians(FRONT_FOV_DEG / 2.0)
        num_rays = len(msg.ranges)

        # Index for a given angle: i = (angle - angle_min) / angle_increment
        # Front-left sector: 0 → +half_fov
        # Front-right sector: (2π - half_fov) → 2π
        idx_left = int(half_fov / msg.angle_increment) if msg.angle_increment > 0 else 0
        idx_right_start = max(0, num_rays - idx_left)

        front_ranges = []
        for i in range(0, min(idx_left, num_rays)):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                front_ranges.append(r)
        for i in range(idx_right_start, num_rays):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                front_ranges.append(r)

        if front_ranges:
            min_dist = min(front_ranges)
            self._obstacle_ahead = min_dist < OBSTACLE_DIST
        else:
            self._obstacle_ahead = False

    # ------------------------------------------------------------------
    # Camera callback — ball detection via HSV thresholding
    # ------------------------------------------------------------------

    def _on_image(self, msg: Image):
        if self._state == State.STOPPED:
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
        if red_pixels >= MIN_BALL_PIXELS:
            self._on_ball_found(red_pixels)

    def _on_ball_found(self, pixel_count: int):
        self._stop_motion()
        self._state = State.STOPPED
        self._timer.cancel()
        self.get_logger().info(
            f"[BallSearcher] Ball found! Stopping search. "
            f"(red pixel area: {pixel_count} px)"
        )

    # ------------------------------------------------------------------
    # Movement state machine (runs at 10 Hz via timer)
    # ------------------------------------------------------------------

    def _tick(self):
        if self._state == State.STOPPED:
            return

        self._elapsed += self._dt

        if self._state == State.MOVE_FORWARD:
            # Check lidar before moving
            if self._obstacle_ahead:
                self.get_logger().info(
                    "Obstacle detected ahead — turning to avoid"
                )
                self._elapsed = 0.0
                self._target_duration = TURN_ANGLE / TURN_SPEED
                self._state = State.AVOID_TURN
                return

            self._publish_cmd(linear=LINEAR_SPEED, angular=0.0)
            if self._elapsed >= self._target_duration:
                self._transition_to_turn()

        elif self._state == State.TURN:
            self._publish_cmd(linear=0.0, angular=TURN_SPEED)
            if self._elapsed >= self._target_duration:
                self._transition_to_move()

        elif self._state == State.AVOID_TURN:
            # Turn in place until the front is clear
            self._publish_cmd(linear=0.0, angular=TURN_SPEED)
            if self._elapsed >= self._target_duration and not self._obstacle_ahead:
                self.get_logger().info("Path clear — resuming forward")
                self._transition_to_move()

    def _transition_to_turn(self):
        self._legs_done += 1
        # Increase leg length every two completed legs
        if self._legs_done % 2 == 0:
            self._leg_length += LEG_INCREMENT

        self._elapsed = 0.0
        self._target_duration = TURN_ANGLE / TURN_SPEED
        self._state = State.TURN
        self.get_logger().debug(
            f"Turning 90° (leg #{self._legs_done} done, "
            f"next leg = {self._leg_length:.1f} m)"
        )

    def _transition_to_move(self):
        self._elapsed = 0.0
        self._target_duration = self._leg_length / LINEAR_SPEED
        self._state = State.MOVE_FORWARD
        self.get_logger().debug(f"Moving forward {self._leg_length:.1f} m")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    def _stop_motion(self):
        self._publish_cmd(linear=0.0, angular=0.0)


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
        node._stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
