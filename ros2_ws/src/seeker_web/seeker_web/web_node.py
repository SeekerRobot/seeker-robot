"""
web_node.py — All-in-one browser controller for the Seeker robot.

Serves an HTML dashboard and bridges the browser to ROS 2 / micro-ROS:

  REST
    GET  /                     → index.html
    GET  /static/<file>        → static assets
    GET  /api/config           → { mcu_ip, cam_url, mic_url }
    POST /api/cmd_vel          → body {vx,vy,wz} → geometry_msgs/Twist on /cmd_vel
    POST /api/stop             → publishes zero Twist on /cmd_vel
    POST /api/tts              → body {text} → std_msgs/String on /audio_tts_input
    POST /api/play_wav         → body {path} → std_msgs/String on /audio_play_file

  WebSocket  /ws
    Server → client
      {"type":"status",  "battery":12.3, "heartbeat":42}
      {"type":"imu",     "ori":[w,x,y,z], "ang":[x,y,z], "acc":[x,y,z]}
      {"type":"lidar",   "angles":[…], "ranges":[…]}   (downsampled)
      {"type":"log",     "text":"…"}
    Client → server (same shape as REST — use whichever is convenient)
      {"type":"cmd_vel", "vx":0.1, "vy":0, "wz":0}
      {"type":"stop"}
      {"type":"tts", "text":"hello"}

The MCU's MJPEG and PCM endpoints are exposed directly on the robot IP, so the
browser pulls them from http://<mcu_ip>:80/cam and :81/audio. That avoids
re-encoding and keeps this node light.

Launch:
  ros2 launch seeker_web web.launch.py
    mcu_ip:=192.168.1.50   http_port:=8080
"""

import asyncio
import json
import os
import threading
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from aiohttp import WSMsgType, web
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Int32, String

# QoS profile matching the MCU-side micro-ROS BEST_EFFORT publishers.
BEST_EFFORT = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Matches seeker_tts/tts_node.py publisher — late joiners get the current
# state immediately.
SPEAKER_ACTIVE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class Telemetry:
    """Thread-safe latest-value cache. The ROS thread writes, the web thread
    reads. Last-writer-wins; the WebSocket blast loop snapshots on each tick."""

    def __init__(self):
        self._lock = threading.Lock()
        self.battery: Optional[float] = None
        self.heartbeat: Optional[int] = None
        self.imu: Optional[dict] = None
        self.lidar: Optional[dict] = None
        self.speaker_active: Optional[bool] = None
        self.speaker_active_dirty: bool = False
        # Log lines accumulate in a bounded ring so we can ship whatever
        # arrived since the last WS tick without holding locks on the ROS
        # side for long.
        self.pending_logs: list[str] = []
        self.MAX_PENDING_LOGS = 200

    def push_log(self, line: str):
        with self._lock:
            self.pending_logs.append(line)
            if len(self.pending_logs) > self.MAX_PENDING_LOGS:
                # Drop oldest when the browser is slow.
                self.pending_logs = self.pending_logs[-self.MAX_PENDING_LOGS :]

    def drain_logs(self) -> list[str]:
        with self._lock:
            out = self.pending_logs
            self.pending_logs = []
            return out

    def snapshot_status(self) -> dict:
        with self._lock:
            return {
                "type": "status",
                "battery": self.battery,
                "heartbeat": self.heartbeat,
            }

    def snapshot_imu(self) -> Optional[dict]:
        with self._lock:
            return self.imu

    def snapshot_lidar(self) -> Optional[dict]:
        with self._lock:
            return self.lidar

    def push_speaker_active(self, active: bool):
        with self._lock:
            if self.speaker_active == active:
                return
            self.speaker_active = active
            self.speaker_active_dirty = True

    def pop_speaker_active(self) -> Optional[bool]:
        """Returns the current value only when it has changed since the last
        pop. Producers use this to send edge-triggered WS updates."""
        with self._lock:
            if not self.speaker_active_dirty:
                return None
            self.speaker_active_dirty = False
            return self.speaker_active


class SeekerWebNode(Node):
    def __init__(self, tel: Telemetry):
        super().__init__("seeker_web_node")
        self.tel = tel

        self.declare_parameter("mcu_ip", "192.168.1.50")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("cam_port", 80)
        self.declare_parameter("mic_port", 81)
        self.declare_parameter("lidar_max_points", 180)
        # WS push rates — seconds between frames (0 disables the stream).
        self.declare_parameter("status_rate_s", 0.1)   # 10 Hz
        self.declare_parameter("imu_rate_s", 0.05)     # 20 Hz
        self.declare_parameter("lidar_rate_s", 0.2)    # 5 Hz
        self.declare_parameter("log_rate_s", 0.1)      # 10 Hz
        # Whitelist of directory prefixes that /api/play_wav will accept. An
        # empty list means "allow anything" (local-dev escape hatch) — a
        # warning is logged at startup if left empty.
        self.declare_parameter(
            "play_wav_allow_roots",
            [
                "/tmp",
                os.path.expanduser(
                    "~/ros2_workspaces/install/seeker_tts/share"
                ),
            ],
        )
        self._play_wav_roots = [
            os.path.realpath(os.path.expanduser(p))
            for p in self.get_parameter("play_wav_allow_roots")
            .get_parameter_value()
            .string_array_value
        ]
        if not self._play_wav_roots:
            self.get_logger().warn(
                "play_wav_allow_roots is empty — /api/play_wav will accept "
                "ANY path. Set the param before exposing this node off-LAN."
            )

        # --- Subscriptions from MCU ---
        self.create_subscription(
            Float32, "/mcu/battery_voltage", self._on_battery, BEST_EFFORT
        )
        self.create_subscription(
            Int32, "/mcu/heartbeat", self._on_heartbeat, BEST_EFFORT
        )
        self.create_subscription(Imu, "/mcu/imu", self._on_imu, BEST_EFFORT)
        self.create_subscription(
            LaserScan, "/mcu/scan", self._on_scan, BEST_EFFORT
        )
        self.create_subscription(String, "/mcu/log", self._on_log, BEST_EFFORT)
        # Drives the browser's mic-element mute via the WS. Latched QoS mirrors
        # seeker_tts — late joiners get the current state without waiting.
        self.create_subscription(
            Bool,
            "/audio_speaker_active",
            self._on_speaker_active,
            SPEAKER_ACTIVE_QOS,
        )

        # --- Publishers to MCU / host nodes ---
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tts_pub = self.create_publisher(String, "/audio_tts_input", 10)
        self.play_file_pub = self.create_publisher(String, "/audio_play_file", 10)

    # ---- Callbacks -----------------------------------------------------

    def _on_battery(self, msg: Float32):
        with self.tel._lock:
            self.tel.battery = float(msg.data)

    def _on_heartbeat(self, msg: Int32):
        with self.tel._lock:
            self.tel.heartbeat = int(msg.data)

    def _on_imu(self, msg: Imu):
        with self.tel._lock:
            self.tel.imu = {
                "type": "imu",
                "ori": [
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ],
                "ang": [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                "acc": [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
            }

    def _on_scan(self, msg: LaserScan):
        # Downsample to lidar_max_points so the WebSocket doesn't flood the
        # browser with 720-pt scans every 150 ms.
        max_pts = (
            self.get_parameter("lidar_max_points").get_parameter_value().integer_value
        )
        n = len(msg.ranges)
        if n == 0:
            return
        step = max(1, n // max_pts)
        angles = []
        ranges = []
        for i in range(0, n, step):
            angles.append(msg.angle_min + i * msg.angle_increment)
            ranges.append(float(msg.ranges[i]))
        with self.tel._lock:
            self.tel.lidar = {"type": "lidar", "angles": angles, "ranges": ranges}

    def _on_log(self, msg: String):
        self.tel.push_log(msg.data)

    def _on_speaker_active(self, msg: Bool):
        self.tel.push_speaker_active(bool(msg.data))

    def is_play_wav_allowed(self, path: str) -> bool:
        """True if `path` (after realpath) sits under one of the allow-roots.
        Empty allow-list means "permit everything" — dev-only escape hatch."""
        if not self._play_wav_roots:
            return True
        resolved = os.path.realpath(os.path.expanduser(path))
        for root in self._play_wav_roots:
            # `+ os.sep` guards against /tmpfoo sneaking past a /tmp root.
            if resolved == root or resolved.startswith(root + os.sep):
                return True
        return False

    # ---- Commands (called from the aiohttp thread) ---------------------

    def publish_cmd_vel(self, vx: float, vy: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(wz)
        self.cmd_vel_pub.publish(t)

    def publish_tts(self, text: str):
        m = String()
        m.data = str(text)
        self.tts_pub.publish(m)

    def publish_play_file(self, path: str):
        m = String()
        m.data = str(path)
        self.play_file_pub.publish(m)


# ---------------------------------------------------------------------------
# aiohttp handlers
# ---------------------------------------------------------------------------


def _static_dir() -> Path:
    """Locate the static/ directory both in source trees (for `python -m`)
    and in the installed share/ layout (ament_python)."""
    try:
        share = Path(get_package_share_directory("seeker_web"))
        p = share / "static"
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).parent / "static"


async def handle_index(request: web.Request) -> web.Response:
    path = _static_dir() / "index.html"
    if not path.exists():
        return web.Response(status=500, text="index.html missing")
    return web.FileResponse(path)


async def handle_config(request: web.Request) -> web.Response:
    node: SeekerWebNode = request.app["node"]
    mcu_ip = node.get_parameter("mcu_ip").get_parameter_value().string_value
    cam_port = node.get_parameter("cam_port").get_parameter_value().integer_value
    mic_port = node.get_parameter("mic_port").get_parameter_value().integer_value
    return web.json_response(
        {
            "mcu_ip": mcu_ip,
            "cam_url": f"http://{mcu_ip}:{cam_port}/cam",
            "cam_stream_url": f"http://{mcu_ip}:{cam_port}/stream",
            "mic_url": f"http://{mcu_ip}:{mic_port}/audio",
            "topics": {
                "sub": [
                    "/mcu/battery_voltage",
                    "/mcu/heartbeat",
                    "/mcu/imu",
                    "/mcu/scan",
                    "/mcu/log",
                ],
                "pub": ["/cmd_vel", "/audio_tts_input", "/audio_play_file"],
            },
        }
    )


async def handle_cmd_vel(request: web.Request) -> web.Response:
    node: SeekerWebNode = request.app["node"]
    try:
        body = await request.json()
    except Exception:
        return web.json_response({"ok": False, "error": "invalid json"}, status=400)
    node.publish_cmd_vel(body.get("vx", 0.0), body.get("vy", 0.0), body.get("wz", 0.0))
    return web.json_response({"ok": True})


async def handle_stop(request: web.Request) -> web.Response:
    node: SeekerWebNode = request.app["node"]
    node.publish_cmd_vel(0.0, 0.0, 0.0)
    return web.json_response({"ok": True})


async def handle_tts(request: web.Request) -> web.Response:
    node: SeekerWebNode = request.app["node"]
    try:
        body = await request.json()
    except Exception:
        return web.json_response({"ok": False, "error": "invalid json"}, status=400)
    text = body.get("text", "")
    if not text:
        return web.json_response({"ok": False, "error": "empty text"}, status=400)
    node.publish_tts(text)
    return web.json_response({"ok": True})


async def handle_play_wav(request: web.Request) -> web.Response:
    node: SeekerWebNode = request.app["node"]
    try:
        body = await request.json()
    except Exception:
        return web.json_response({"ok": False, "error": "invalid json"}, status=400)
    path = body.get("path", "")
    if not path:
        return web.json_response({"ok": False, "error": "empty path"}, status=400)
    if not node.is_play_wav_allowed(path):
        node.get_logger().warn(f"Rejecting play_wav path outside allowlist: {path}")
        return web.json_response(
            {"ok": False, "error": "path not in allow-list"}, status=403
        )
    node.publish_play_file(path)
    return web.json_response({"ok": True})


async def handle_ws(request: web.Request) -> web.WebSocketResponse:
    """Full-duplex. Server pushes telemetry at 10 Hz (status), 20 Hz (imu),
    ~6 Hz (lidar as available). Client sends command JSON messages."""
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    node: SeekerWebNode = request.app["node"]
    tel: Telemetry = request.app["tel"]

    status_rate = (
        node.get_parameter("status_rate_s").get_parameter_value().double_value
    )
    imu_rate = node.get_parameter("imu_rate_s").get_parameter_value().double_value
    lidar_rate = (
        node.get_parameter("lidar_rate_s").get_parameter_value().double_value
    )
    log_rate = node.get_parameter("log_rate_s").get_parameter_value().double_value

    async def producer():
        last_status = last_imu = last_lidar = last_log = 0.0
        while not ws.closed:
            now = asyncio.get_event_loop().time()
            # Speaker-active fires on edge, not on cadence — the browser
            # needs to mute BEFORE playback starts, not on the next 100 ms
            # tick.
            spk = tel.pop_speaker_active()
            if spk is not None:
                await ws.send_json({"type": "speaker_active", "active": spk})
            if status_rate > 0 and now - last_status >= status_rate:
                await ws.send_json(tel.snapshot_status())
                last_status = now
            if imu_rate > 0 and now - last_imu >= imu_rate:
                imu = tel.snapshot_imu()
                if imu is not None:
                    await ws.send_json(imu)
                last_imu = now
            if lidar_rate > 0 and now - last_lidar >= lidar_rate:
                lidar = tel.snapshot_lidar()
                if lidar is not None:
                    await ws.send_json(lidar)
                last_lidar = now
            if log_rate > 0 and now - last_log >= log_rate:
                for line in tel.drain_logs():
                    await ws.send_json({"type": "log", "text": line})
                last_log = now
            await asyncio.sleep(0.02)

    producer_task = asyncio.create_task(producer())
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except Exception:
                continue
            kind = data.get("type")
            if kind == "cmd_vel":
                node.publish_cmd_vel(
                    data.get("vx", 0.0), data.get("vy", 0.0), data.get("wz", 0.0)
                )
            elif kind == "stop":
                node.publish_cmd_vel(0.0, 0.0, 0.0)
            elif kind == "tts":
                text = data.get("text", "")
                if text:
                    node.publish_tts(text)
            elif kind == "play_wav":
                path = data.get("path", "")
                if path:
                    node.publish_play_file(path)
    finally:
        producer_task.cancel()
        try:
            await producer_task
        except asyncio.CancelledError:
            pass
    return ws


# ---------------------------------------------------------------------------
# Entry point — runs rclpy on a background thread, asyncio on the main thread
# ---------------------------------------------------------------------------


def _spin_ros(node: SeekerWebNode, stop_event: threading.Event):
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)


async def _amain(node: SeekerWebNode, tel: Telemetry, host: str, port: int):
    app = web.Application()
    app["node"] = node
    app["tel"] = tel
    app.router.add_get("/", handle_index)
    app.router.add_static("/static/", _static_dir(), show_index=False)
    app.router.add_get("/api/config", handle_config)
    app.router.add_post("/api/cmd_vel", handle_cmd_vel)
    app.router.add_post("/api/stop", handle_stop)
    app.router.add_post("/api/tts", handle_tts)
    app.router.add_post("/api/play_wav", handle_play_wav)
    app.router.add_get("/ws", handle_ws)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    node.get_logger().info(f"seeker_web listening on http://{host}:{port}")
    # Block forever — aiohttp owns the main thread event loop.
    while True:
        await asyncio.sleep(3600)


def main(args=None):
    rclpy.init(args=args)
    tel = Telemetry()
    node = SeekerWebNode(tel)

    host = os.environ.get("SEEKER_WEB_HOST", "0.0.0.0")
    port = node.get_parameter("http_port").get_parameter_value().integer_value

    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_ros, args=(node, stop_event), daemon=True
    )
    spin_thread.start()

    try:
        asyncio.run(_amain(node, tel, host, port))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
