#!/usr/bin/env python3
"""fake_mcu_node — simulation stand-in for the Seeker hexapod MCU.

Implements a joint-space tripod gait for the 2-DOF (hip yaw + knee pitch)
hexapod. No IK required — locomotion comes entirely from sweeping the hip.

== Gait geometry (why LEG_SIDE matters) ==

For a leg mounted at angle α from body +X, the foot's forward (X) displacement
relative to the hip is:   foot_x = L_eff * cos(α + θ_hip)

  Left legs  (α > 0, e.g. ML α=90°):  positive θ moves foot toward -X (rear)
  Right legs (α < 0, e.g. MR α=-90°): positive θ moves foot toward +X (front)

So the two sides have OPPOSITE sign conventions. Multiplying by LEG_SIDE (+1
for left, -1 for right) unifies them: a positive leg_stride always causes the
foot to be in the forward (+X) position at stance start, and the body pushes
forward during the hip sweep.

  Stance: hip = LEG_SIDE * stride * (2*s − 1)   s: 0→1
          foot goes from forward (+X) to rear (−X) → body pushed forward
  Swing:  hip = LEG_SIDE * stride * (1 − 2*ss)  ss: smoothstep(0→1)
          hip returns from rear to forward while foot is lifted

== Turning ==

For CCW turn (wz > 0), the right side needs a larger stride (more forward push)
than the left side:   leg_stride = vx_stride − LEG_SIDE[i] * turn_amp

Tuning constants are at the top of this file.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from mcu_msgs.msg import HexapodCmd

# ── Gait tuning ───────────────────────────────────────────────────────────────

CYCLE_TIME   = 0.8              # full tripod cycle duration (seconds)
NEUTRAL_KNEE = math.radians(30.0)   # knee angle in standing stance — body at ~109mm above ground
LIFT_KNEE    = math.radians(88.0)   # knee angle at peak of swing — ~66mm clearance above ground
# NOTE: In this URDF, knee=0° = tibia vertical (foot at maximum depth),
#       knee=90° = tibia horizontal (foot at minimum depth / maximum lift).
#       LIFT_KNEE must be LARGER than NEUTRAL_KNEE to actually lift the foot.
#       Increasing NEUTRAL_KNEE raises the body (e.g. 45°=66mm depth, 60°=50mm depth, 70°=37mm depth).
#       base_footprint_joint z in the URDF must match: coxa_len + tibia_len*cos(NEUTRAL_KNEE) + leg_r
#       (the +leg_r accounts for the sphere foot collision — contact when sphere centre is leg_r above ground).
STRIDE_HIP   = math.radians(45.0)   # maximum hip sweep amplitude per side (degrees)
VX_MAX       = 0.4              # m/s — velocity that maps to full STRIDE_HIP
WZ_MAX       = 1.2              # rad/s — yaw rate that maps to half STRIDE_HIP
DEAD_BAND    = 0.005            # m/s — below this magnitude treat as stopped

# Body height control (t / b keys via linear.z)
# lower knee angle → more vertical tibia → foot reaches further → body sits higher
HEIGHT_RATE      = math.radians(25.0)   # knee angle change rate (rad/s) when t/b held
NEUTRAL_KNEE_MIN = math.radians(5.0)    # tallest allowed stance
NEUTRAL_KNEE_MAX = LIFT_KNEE - math.radians(10.0)  # shortest — keeps ≥10° swing clearance

# Joint limits (radians) — must match URDF
HIP_MIN  = -1.047   # −60°
HIP_MAX  =  1.047   # +60°
KNEE_MIN =  0.000   #  0°
KNEE_MAX =  1.571   # +90°

# ── Leg configuration ─────────────────────────────────────────────────────────

# Leg order: 0=FL, 1=FR, 2=ML, 3=MR, 4=RL, 5=RR
LEG_NAMES = ['fl', 'fr', 'ml', 'mr', 'rl', 'rr']

# +1 = left side of body, −1 = right side (see geometry explanation above)
LEG_SIDE = [1, -1, 1, -1, 1, -1]

# Tripod groups — A starts in stance (phase=0.0), B starts in swing (phase=0.5)
GROUP_A = [0, 3, 4]   # FL, MR, RL
GROUP_B = [1, 2, 5]   # FR, ML, RR

# Dance: override /cmd_vel with a yaw oscillation so the hexapod spins in place
DANCE_DURATION = 3.0              # seconds
DANCE_WZ_AMPLITUDE = 1.2           # rad/s peak
DANCE_WZ_FREQ = 1.0                # Hz


# ── Node ──────────────────────────────────────────────────────────────────────

class FakeMcuNode(Node):

    def __init__(self):
        super().__init__('fake_mcu_node')

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._vz = 0.0
        self._neutral_knee = NEUTRAL_KNEE   # adjustable at runtime via t/b keys

        # Dance override: when active, ignore /cmd_vel wz and oscillate yaw
        self._dance_until = 0.0     # wall-clock-style node time; 0 = not dancing
        self._dance_start = 0.0

        # Per-leg gait phase ∈ [0, 1)
        self._phase = [0.0 if i in GROUP_A else 0.5 for i in range(6)]

        self._js_pub = self.create_publisher(JointState, '/mcu/joint_states', 10)

        self._gz_pubs = {}
        for name in LEG_NAMES:
            for jtype in ('hip', 'knee'):
                topic = f'/seeker/joint/leg_{name}_{jtype}/cmd_pos'
                self._gz_pubs[f'{name}_{jtype}'] = \
                    self.create_publisher(Float64, topic, 10)


        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(HexapodCmd, '/mcu/hexapod_cmd', self._hexapod_cmd_cb, 10)

        self._last_t = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(1.0 / 100, self._update)

        self.get_logger().info('fake_mcu_node ready')

    def _cmd_vel_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._wz = msg.angular.z
        self._vz = msg.linear.z

    def _hexapod_cmd_cb(self, msg: HexapodCmd):
        if msg.mode == HexapodCmd.MODE_DANCE:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._dance_start = now
            self._dance_until = now + DANCE_DURATION
            self.get_logger().info(f'fake_mcu: dancing for {DANCE_DURATION:.1f}s')
        else:
            # Any other gait command cancels the dance
            self._dance_until = 0.0

    def _update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = min(max(now - self._last_t, 0.0), 0.1)
        self._last_t = now

        vx, wz = self._vx, self._wz
        if self._dance_until > 0.0:
            if now < self._dance_until:
                phase = 2.0 * math.pi * DANCE_WZ_FREQ * (now - self._dance_start)
                vx = 0.0
                wz = DANCE_WZ_AMPLITUDE * math.sin(phase)
            else:
                self._dance_until = 0.0
        moving = (abs(vx) + abs(self._vy) + abs(wz)) > DEAD_BAND

        # t/b keys → linear.z: positive = raise body (decrease knee angle),
        #                        negative = lower body (increase knee angle)
        if abs(self._vz) > DEAD_BAND:
            self._neutral_knee -= math.copysign(HEIGHT_RATE, self._vz) * dt
            self._neutral_knee  = max(NEUTRAL_KNEE_MIN,
                                      min(NEUTRAL_KNEE_MAX, self._neutral_knee))

        nk = self._neutral_knee   # local alias used throughout the loop

        # Normalised velocity magnitudes, clamped to [−1, 1]
        vx_stride  = STRIDE_HIP * max(-1.0, min(1.0, vx / VX_MAX))
        turn_amp   = STRIDE_HIP * 0.5 * max(-1.0, min(1.0, wz / WZ_MAX))

        hip_angles  = [0.0] * 6
        knee_angles = [nk]  * 6

        for i in range(6):
            if moving:
                self._phase[i] += dt / (CYCLE_TIME * 0.5)
                if self._phase[i] >= 1.0:
                    self._phase[i] -= 1.0
            else:
                # When stopped, snap all legs to neutral stance — hips centred,
                # knees at current height target. Reset phases for clean walk start.
                self._phase[i] = 0.0 if i in GROUP_A else 0.5
                hip_angles[i]  = 0.0
                knee_angles[i] = nk
                continue

            # Per-leg stride: right side gets more for CCW turn (wz > 0)
            leg_stride = vx_stride - LEG_SIDE[i] * turn_amp

            side = LEG_SIDE[i]
            flying = self._phase[i] >= 0.5

            if flying:
                # Swing: foot in air, hip returns from rear to forward
                s  = (self._phase[i] - 0.5) / 0.5   # 0 → 1
                ss = smoothstep(s)
                hip  = side * leg_stride * (1.0 - 2.0 * ss)
                # Raise quickly (first 30%), hold high (middle 40%), lower slowly (last 30%)
                if s < 0.3:
                    lift_t = s / 0.3
                elif s < 0.7:
                    lift_t = 1.0
                else:
                    lift_t = 1.0 - (s - 0.7) / 0.3
                knee = nk - (nk - LIFT_KNEE) * lift_t
            else:
                # Stance: foot planted, hip sweeps foot from forward to rear
                s   = self._phase[i] / 0.5            # 0 → 1
                hip = side * leg_stride * (2.0 * s - 1.0)
                knee = nk

            hip_angles[i]  = max(HIP_MIN,  min(HIP_MAX,  hip))
            knee_angles[i] = max(KNEE_MIN, min(KNEE_MAX, knee))

        self._publish(hip_angles, knee_angles)

    def _publish(self, hip_angles, knee_angles):
        now = self.get_clock().now().to_msg()
        js  = JointState()
        js.header.stamp = now

        for i, name in enumerate(LEG_NAMES):
            hip  = hip_angles[i]
            knee = knee_angles[i]

            js.name.append(f'leg_{name}_hip_joint')
            js.position.append(hip)
            js.velocity.append(0.0)
            js.effort.append(0.0)

            js.name.append(f'leg_{name}_knee_joint')
            js.position.append(knee)
            js.velocity.append(0.0)
            js.effort.append(0.0)

            h = Float64(); h.data = hip
            k = Float64(); k.data = knee
            self._gz_pubs[f'{name}_hip'].publish(h)
            self._gz_pubs[f'{name}_knee'].publish(k)

        self._js_pub.publish(js)


def smoothstep(s: float) -> float:
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMcuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
