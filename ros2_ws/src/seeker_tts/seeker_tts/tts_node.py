"""ROS 2 node: TTS + WAV file playback → HTTP audio stream for ESP32."""

import math
import os
import queue
import struct
import threading
import wave
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    import requests
except ImportError:
    requests = None

try:
    from scipy.signal import lfilter as _scipy_lfilter
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


def _apply_lowshelf(pcm: bytes, sample_rate: int, freq_hz: float, gain_db: float) -> bytes:
    """Biquad low-shelf EQ (Audio EQ Cookbook, S=1).

    Attenuates (gain_db < 0) or boosts (gain_db > 0) frequencies below
    freq_hz. No-op when gain_db is within 0.1 dB of 0.
    """
    if abs(gain_db) < 0.1:
        return pcm

    A      = 10.0 ** (gain_db / 40.0)
    w0     = 2.0 * math.pi * freq_hz / sample_rate
    cos_w0 = math.cos(w0)
    alpha  = math.sin(w0) / math.sqrt(2.0)   # shelf slope S = 1
    sq_A   = math.sqrt(A)

    b0 =    A * ((A+1) - (A-1)*cos_w0 + 2*sq_A*alpha)
    b1 =  2*A * ((A-1) - (A+1)*cos_w0)
    b2 =    A * ((A+1) - (A-1)*cos_w0 - 2*sq_A*alpha)
    a0 =        (A+1) + (A-1)*cos_w0 + 2*sq_A*alpha
    a1 =   -2 * ((A-1) + (A+1)*cos_w0)
    a2 =        (A+1) + (A-1)*cos_w0 - 2*sq_A*alpha

    b = np.array([b0/a0, b1/a0, b2/a0])
    a = np.array([1.0,   a1/a0, a2/a0])

    samples = np.frombuffer(pcm, dtype="<i2").astype(np.float64)

    if _HAS_SCIPY:
        filtered = _scipy_lfilter(b, a, samples)
    else:
        # Direct-form II transposed — sequential but allocation-free.
        filtered = np.empty_like(samples)
        z1 = z2 = 0.0
        for i, x in enumerate(samples):
            y      = b[0]*x + z1
            z1     = b[1]*x - a[1]*y + z2
            z2     = b[2]*x - a[2]*y
            filtered[i] = y

    return np.clip(filtered, -32768, 32767).astype("<i2").tobytes()


class TtsNode(Node):
    def __init__(self):
        super().__init__("tts_node")

        self.declare_parameter("fish_api_key", "")
        self.declare_parameter("fish_reference_id", "")
        self.declare_parameter("fish_model", "s2-pro")
        self.declare_parameter("serve_port", 8383)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("eq_bass_hz", 300.0)
        self.declare_parameter("eq_bass_db", 0.0)
        # Hold /audio_speaker_active=True for this many seconds after the last
        # clip drains, so downstream mic gates don't reopen during the ESP32
        # speaker's tail/I2S flush window.
        self.declare_parameter("idle_release_delay_s", 4.0)

        self._api_key = (
            self.get_parameter("fish_api_key").get_parameter_value().string_value
            or os.environ.get("FISH_API_KEY", "")
        )
        self._ref_id = (
            self.get_parameter("fish_reference_id")
            .get_parameter_value()
            .string_value
            or os.environ.get("FISH_REFERENCE_ID", "")
        )
        self._model = (
            self.get_parameter("fish_model").get_parameter_value().string_value
        )
        self._serve_port = (
            self.get_parameter("serve_port")
            .get_parameter_value()
            .integer_value
        )
        self._sample_rate = (
            self.get_parameter("sample_rate")
            .get_parameter_value()
            .integer_value
        )
        self._eq_bass_hz = (
            self.get_parameter("eq_bass_hz").get_parameter_value().double_value
        )
        self._eq_bass_db = (
            self.get_parameter("eq_bass_db").get_parameter_value().double_value
        )
        self._idle_release_delay_s = (
            self.get_parameter("idle_release_delay_s")
            .get_parameter_value()
            .double_value
        )

        if not self._api_key:
            self.get_logger().warn(
                "No Fish Audio API key. Set FISH_API_KEY env var or fish_api_key param. "
                "TTS will be unavailable; file playback still works."
            )
        if requests is None:
            self.get_logger().warn("python3-requests not installed — TTS disabled")

        # Single-slot queue: only one audio clip streams at a time.
        # TTS blocks the executor during API call so it naturally goes first.
        # File playback drops if the queue is full (TTS has priority).
        self._audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=1)

        # Edge-triggered "speaker is currently emitting audio" signal. Host-
        # side mic consumers (seeker_web → browser mic element) subscribe and
        # mute while this is true. TRANSIENT_LOCAL + depth 1 so a late-joining
        # subscriber gets the current state immediately rather than waiting
        # for the next edge.
        self._speaker_active_lock = threading.Lock()
        self._speaker_active = False
        self._idle_release_timer: threading.Timer | None = None
        speaker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._speaker_active_pub = self.create_publisher(
            Bool, "/audio_speaker_active", speaker_qos
        )
        self._publish_speaker_active(False)

        self._start_http_server()

        self._tts_sub = self.create_subscription(
            String, "/audio_tts_input", self._on_tts, 10
        )
        self._file_sub = self.create_subscription(
            String, "/audio_play_file", self._on_play_file, 10
        )
        self.get_logger().info(
            f"TTS node ready — /audio_tts_input (TTS), /audio_play_file (WAV), "
            f"serving on :{self._serve_port}/audio_out"
        )

    # ---- Speaker-active edge publisher ---------------------------------------

    def _publish_speaker_active(self, active: bool):
        with self._speaker_active_lock:
            # Cancel any pending delayed-release whenever state is touched,
            # since either path supersedes it (True = re-engage, False = will
            # re-arm fresh below).
            if self._idle_release_timer is not None:
                self._idle_release_timer.cancel()
                self._idle_release_timer = None

            if active:
                if self._speaker_active:
                    return
                self._speaker_active = True
                publish_now = True
            else:
                if not self._speaker_active:
                    return
                # Defer the False edge so the ESP32 I2S tail can flush before
                # the mic gate reopens.
                if self._idle_release_delay_s > 0.0:
                    self._idle_release_timer = threading.Timer(
                        self._idle_release_delay_s, self._release_speaker_now
                    )
                    self._idle_release_timer.daemon = True
                    self._idle_release_timer.start()
                    return
                self._speaker_active = False
                publish_now = True

        if publish_now:
            msg = Bool()
            msg.data = active
            self._speaker_active_pub.publish(msg)

    def _release_speaker_now(self):
        with self._speaker_active_lock:
            self._idle_release_timer = None
            if not self._speaker_active:
                return
            self._speaker_active = False
        msg = Bool()
        msg.data = False
        self._speaker_active_pub.publish(msg)

    # ---- TTS callback --------------------------------------------------------

    def _on_tts(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"TTS request: {text[:80]}")

        pcm = self._fish_tts(text)
        if pcm is None:
            return

        pcm = _apply_lowshelf(pcm, self._sample_rate, self._eq_bass_hz, self._eq_bass_db)
        # Flip the mute gate BEFORE the put so any in-flight /audio_out poll
        # that wins the race sees active=True before it starts streaming.
        self._publish_speaker_active(True)
        # Blocking put — waits until previous audio is consumed.
        self._audio_queue.put(pcm)

    # ---- File playback callback ----------------------------------------------

    def _on_play_file(self, msg: String):
        filepath = msg.data.strip()
        if not filepath:
            return

        self.get_logger().info(f"File play request: {filepath}")

        pcm = self._load_wav(filepath)
        if pcm is None:
            return

        pcm = _apply_lowshelf(pcm, self._sample_rate, self._eq_bass_hz, self._eq_bass_db)
        try:
            self._audio_queue.put_nowait(pcm)
        except queue.Full:
            self.get_logger().warn(
                f"Dropped file playback — TTS audio still streaming"
            )
            return
        self._publish_speaker_active(True)

    # ---- Fish Audio TTS ------------------------------------------------------

    def _fish_tts(self, text: str) -> bytes | None:
        if requests is None or not self._api_key:
            return None

        headers = {
            "Authorization": f"Bearer {self._api_key}",
            "Content-Type": "application/json",
            "model": self._model,
        }
        body = {
            "text": text,
            "format": "pcm",
            "sample_rate": self._sample_rate,
        }
        if self._ref_id:
            body["reference_id"] = self._ref_id

        try:
            resp = requests.post(
                "https://api.fish.audio/v1/tts",
                headers=headers,
                json=body,
                stream=True,
                timeout=30,
            )
            resp.raise_for_status()
        except requests.RequestException as e:
            self.get_logger().error(f"Fish Audio request failed: {e}")
            return None

        pcm = b"".join(resp.iter_content(chunk_size=4096))
        duration_s = len(pcm) / (self._sample_rate * 2)
        self.get_logger().info(
            f"Got {len(pcm)} bytes PCM ({duration_s:.1f}s audio)"
        )
        return pcm

    # ---- WAV loader ----------------------------------------------------------

    def _load_wav(self, filepath: str) -> bytes | None:
        if not os.path.isfile(filepath):
            self.get_logger().error(f"File not found: {filepath}")
            return None

        try:
            with wave.open(filepath, "rb") as wf:
                nchannels = wf.getnchannels()
                sampwidth = wf.getsampwidth()
                framerate = wf.getframerate()
                frames = wf.readframes(wf.getnframes())
        except wave.Error as e:
            self.get_logger().error(f"Invalid WAV file: {e}")
            return None

        # Convert to 16-bit mono at target sample rate.
        pcm = self._convert_pcm(frames, nchannels, sampwidth, framerate)
        duration_s = len(pcm) / (self._sample_rate * 2)
        self.get_logger().info(
            f"Loaded {filepath} → {len(pcm)} bytes PCM ({duration_s:.1f}s)"
        )
        return pcm

    def _convert_pcm(
        self, frames: bytes, nchannels: int, sampwidth: int, framerate: int
    ) -> bytes:
        """Convert WAV frames to 16-bit mono at self._sample_rate."""
        # Step 1: convert to 16-bit if needed.
        if sampwidth == 1:
            # 8-bit unsigned → 16-bit signed
            samples = [((b - 128) << 8) for b in frames]
            frames = struct.pack(f"<{len(samples)}h", *samples)
            sampwidth = 2
        elif sampwidth == 3:
            # 24-bit signed → 16-bit signed (drop low byte)
            out = bytearray()
            for i in range(0, len(frames), 3):
                out.extend(frames[i + 1 : i + 3])
            frames = bytes(out)
            sampwidth = 2
        elif sampwidth == 4:
            # 32-bit signed → 16-bit signed
            samples_32 = struct.unpack(f"<{len(frames) // 4}i", frames)
            frames = struct.pack(
                f"<{len(samples_32)}h", *[s >> 16 for s in samples_32]
            )
            sampwidth = 2

        # Step 2: stereo → mono (average channels).
        if nchannels == 2:
            samples_16 = struct.unpack(f"<{len(frames) // 2}h", frames)
            mono = []
            for i in range(0, len(samples_16), 2):
                mono.append((samples_16[i] + samples_16[i + 1]) // 2)
            frames = struct.pack(f"<{len(mono)}h", *mono)
            nchannels = 1
        elif nchannels > 2:
            # Take first channel only.
            samples_16 = struct.unpack(f"<{len(frames) // 2}h", frames)
            mono = samples_16[::nchannels]
            frames = struct.pack(f"<{len(mono)}h", *mono)
            nchannels = 1

        # Step 3: resample if needed (linear interpolation).
        if framerate != self._sample_rate:
            src = struct.unpack(f"<{len(frames) // 2}h", frames)
            ratio = framerate / self._sample_rate
            n_out = int(len(src) / ratio)
            resampled = []
            for i in range(n_out):
                pos = i * ratio
                idx = int(pos)
                frac = pos - idx
                if idx + 1 < len(src):
                    s = src[idx] + frac * (src[idx + 1] - src[idx])
                else:
                    s = src[idx] if idx < len(src) else 0
                s = max(-32768, min(32767, int(s)))
                resampled.append(s)
            frames = struct.pack(f"<{len(resampled)}h", *resampled)

        return frames

    # ---- HTTP server for ESP32 -----------------------------------------------

    def _start_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/audio_out":
                    return self._handle_stream()
                self.send_response(404)
                self.end_headers()

            def _handle_stream(self):
                """One clip per connection. Returns 204 when idle so the
                ESP32 reconnects immediately and tries again. This avoids
                I2S DMA underrun from the persistent-stream design where the
                DMA looped the last buffer while waiting for the next clip."""
                try:
                    data = node._audio_queue.get(timeout=25)
                except queue.Empty:
                    # Confirm idle state — no pending clip means the mic gate
                    # should be off. Cheap no-op when already false.
                    node._publish_speaker_active(False)
                    self.send_response(204)
                    self.end_headers()
                    return

                duration_s = len(data) / (node._sample_rate * 2)
                node.get_logger().info(
                    f"Sending {len(data)} bytes PCM ({duration_s:.1f}s) to client"
                )
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                try:
                    self.wfile.write(data)
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    node.get_logger().warn(
                        "Client disconnected mid-playback; clip lost"
                    )
                finally:
                    # Only lower the gate when the queue is empty — otherwise
                    # the next clip will re-raise it within ms, causing the
                    # mic to flicker unmuted between back-to-back utterances.
                    if node._audio_queue.empty():
                        node._publish_speaker_active(False)

            def log_message(self, format, *args):
                node.get_logger().debug(format % args)

        server = HTTPServer(("0.0.0.0", self._serve_port), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
