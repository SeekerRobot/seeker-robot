"""scripted_cmd_vel — replay a YAML script of forward/turn commands on /cmd_vel.

Forward/backward legs run either for a fixed duration (open-loop) OR until
a target distance is reached on /odom (closed-loop). Turns always close the
loop on yaw from /odom (published by dead_reckoning_odom or EKF, corrected
via slam_toolbox's map→odom TF). Intended for manually driving a scripted
trajectory on the real robot while a mirrored simulation plays the same
/cmd_vel stream — gives a "what-if" preview from start to finish.

YAML format (loaded via `script_path` param):

    steps:
      - {type: forward,  duration: 3.0}          # time-based leg
      - {type: forward,  inches:   24.0}         # closed-loop distance (odom)
      - {type: forward,  meters:   0.5}          # closed-loop distance (odom)
      - {type: turn,     angle_deg: 90.0}
      - {type: backward, inches:   12.0, speed: 0.08}
      - {type: wait,     duration:  1.0}

Params:
  script_path        (required)     — absolute path to the YAML script
  linear_speed       (default 0.10) — default m/s for forward/backward legs
  angular_speed      (default 0.20) — rad/s magnitude for turns
  yaw_tolerance_deg  (default 3.0)  — turn completion tolerance
  distance_tol_m     (default 0.02) — forward/backward distance tolerance
  scale              (default 1.0)  — global multiplier on durations AND
                                      distances (meters/inches). Useful for
                                      scaling a sim-tuned script down for
                                      slower real-robot runs, or running at
                                      half-size in a cramped test area.
  cmd_vel_topic      (default /cmd_vel)
  odom_topic         (default /odom)
  loop               (default False) — restart script on completion
  tick_hz            (default 20.0)
"""

import math

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ScriptedCmdVel(Node):
    def __init__(self) -> None:
        super().__init__("scripted_cmd_vel")

        self.declare_parameter("script_path", "")
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.20)
        self.declare_parameter("yaw_tolerance_deg", 3.0)
        self.declare_parameter("distance_tol_m", 0.02)
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("loop", False)
        self.declare_parameter("tick_hz", 20.0)

        self._script_path = str(self.get_parameter("script_path").value)
        self._v = float(self.get_parameter("linear_speed").value)
        self._w = float(self.get_parameter("angular_speed").value)
        self._yaw_tol = math.radians(
            float(self.get_parameter("yaw_tolerance_deg").value)
        )
        self._dist_tol = float(self.get_parameter("distance_tol_m").value)
        self._scale = float(self.get_parameter("scale").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self._loop = bool(self.get_parameter("loop").value)
        tick_period = 1.0 / float(self.get_parameter("tick_hz").value)

        if not self._script_path:
            raise RuntimeError("scripted_cmd_vel: 'script_path' parameter is required")

        with open(self._script_path) as f:
            doc = yaml.safe_load(f)
        steps = doc.get("steps") if isinstance(doc, dict) else doc
        if not isinstance(steps, list) or not steps:
            raise RuntimeError(f"{self._script_path}: no 'steps' list found")
        self._steps = steps

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._yaw: float | None = None
        self._pos: tuple[float, float] | None = None
        self._idx = 0
        self._state = "INIT"
        self._step_start = 0.0
        self._duration = 0.0
        self._cur_vx = 0.0
        self._target_delta = 0.0          # signed rotation target (rad)
        self._accum_delta = 0.0           # accumulated yaw change (rad)
        self._last_turn_yaw: float | None = None
        self._target_dist = 0.0
        self._start_pos: tuple[float, float] | None = None

        self.create_timer(tick_period, self._tick)

        self.get_logger().info(
            f"scripted_cmd_vel ready — {len(self._steps)} steps from {self._script_path} "
            f"(v={self._v} m/s, w={self._w} rad/s, scale={self._scale}, loop={self._loop})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _publish(self, vx: float, wz: float) -> None:
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self._pub.publish(t)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_step(self) -> None:
        if self._idx >= len(self._steps):
            if self._loop:
                self._idx = 0
                self.get_logger().info("script complete — looping")
            else:
                self.get_logger().info("script complete")
                self._publish(0.0, 0.0)
                self._state = "DONE"
                return

        step = self._steps[self._idx]
        kind = str(step.get("type", "")).lower()

        if kind in ("forward", "backward"):
            sign = 1.0 if kind == "forward" else -1.0
            speed = float(step.get("speed", self._v)) * sign
            self._cur_vx = speed

            # Distance-based (closed-loop on /odom) vs time-based. Priority:
            # inches → meters → duration. Only one applies per step.
            if "inches" in step:
                if self._pos is None:
                    return  # wait for odom
                dist_m = float(step["inches"]) * 0.0254 * self._scale
                self._target_dist = dist_m
                self._start_pos = self._pos
                self._state = "DRIVE_DIST"
                self.get_logger().info(
                    f"[{self._idx}] {kind} {float(step['inches']):.2f}in "
                    f"({dist_m:.3f} m, scale={self._scale}) @ {speed:+.3f} m/s"
                )
            elif "meters" in step:
                if self._pos is None:
                    return
                dist_m = float(step["meters"]) * self._scale
                self._target_dist = dist_m
                self._start_pos = self._pos
                self._state = "DRIVE_DIST"
                self.get_logger().info(
                    f"[{self._idx}] {kind} {dist_m:.3f} m (scale={self._scale}) @ {speed:+.3f} m/s"
                )
            else:
                self._duration = float(step.get("duration", 0.0)) * self._scale
                self._step_start = self._now_s()
                self._state = "DRIVE"
                self.get_logger().info(
                    f"[{self._idx}] {kind} {self._duration:.2f}s (scale={self._scale}) "
                    f"@ {speed:+.3f} m/s"
                )
        elif kind == "turn":
            if self._yaw is None:
                # hold off until first odom arrives
                return
            # Accumulated-delta turn: |target_delta| may exceed 2π (full spins),
            # so we integrate wrapped-yaw increments instead of comparing to
            # an absolute target yaw. Sign = direction (CCW +, CW -).
            self._target_delta = math.radians(float(step.get("angle_deg", 0.0)))
            self._accum_delta = 0.0
            self._last_turn_yaw = self._yaw
            self._state = "TURN"
            self.get_logger().info(
                f"[{self._idx}] turn {math.degrees(self._target_delta):+.1f}° "
                f"(start yaw {math.degrees(self._yaw):+.1f}°)"
            )
        elif kind == "wait":
            self._duration = float(step.get("duration", 0.0)) * self._scale
            self._step_start = self._now_s()
            self._state = "WAIT"
            self.get_logger().info(
                f"[{self._idx}] wait {self._duration:.2f}s (scale={self._scale})"
            )
        else:
            self.get_logger().warn(f"[{self._idx}] unknown step type {kind!r}, skipping")
            self._idx += 1
            self._state = "INIT"

    def _tick(self) -> None:
        if self._state == "DONE":
            self._publish(0.0, 0.0)
            return

        if self._state == "INIT":
            self._start_step()
            return

        now = self._now_s()

        if self._state == "DRIVE":
            if now - self._step_start >= self._duration:
                self._publish(0.0, 0.0)
                self._idx += 1
                self._state = "INIT"
            else:
                self._publish(self._cur_vx, 0.0)
            return

        if self._state == "DRIVE_DIST":
            if self._pos is None or self._start_pos is None:
                self._publish(0.0, 0.0)
                return
            dx = self._pos[0] - self._start_pos[0]
            dy = self._pos[1] - self._start_pos[1]
            travelled = math.hypot(dx, dy)
            if travelled + self._dist_tol >= self._target_dist:
                self._publish(0.0, 0.0)
                self.get_logger().info(
                    f"[{self._idx}] drive-dist done (travelled {travelled:.3f} m)"
                )
                self._idx += 1
                self._state = "INIT"
            else:
                self._publish(self._cur_vx, 0.0)
            return

        if self._state == "WAIT":
            self._publish(0.0, 0.0)
            if now - self._step_start >= self._duration:
                self._idx += 1
                self._state = "INIT"
            return

        if self._state == "TURN":
            if self._yaw is None or self._last_turn_yaw is None:
                self._publish(0.0, 0.0)
                return
            # Integrate wrapped yaw increments — lets |target| exceed 2π.
            step_delta = _wrap_pi(self._yaw - self._last_turn_yaw)
            self._accum_delta += step_delta
            self._last_turn_yaw = self._yaw
            remaining = self._target_delta - self._accum_delta
            if abs(remaining) <= self._yaw_tol:
                self._publish(0.0, 0.0)
                self.get_logger().info(
                    f"[{self._idx}] turn done "
                    f"(rotated {math.degrees(self._accum_delta):+.1f}°, yaw={math.degrees(self._yaw):+.1f}°)"
                )
                self._idx += 1
                self._state = "INIT"
            else:
                wz = self._w if remaining > 0.0 else -self._w
                self._publish(0.0, wz)
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScriptedCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
