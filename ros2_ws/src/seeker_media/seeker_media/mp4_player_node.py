"""
mp4_player_node — Stream an MP4 to the OLED display and ESP32 audio simultaneously.

Video pipeline  : ffmpeg → 128×64 grayscale → SSD1306 framebuffer → HTTP :lcd_serve_port/lcd_out
Audio pipeline  : ffmpeg → 16 kHz 16-bit mono PCM → HTTP :serve_port/audio_out → ESP32

Sync strategy   : Audio is extracted first (sub-second via ffmpeg) and held in a
                  queue.  The video pipeline starts immediately but blocks until the
                  audio HTTP handler fires _audio_connected_event (i.e. the ESP32 has
                  connected and audio bytes are about to flow).  This removes the
                  variable 0–3 s polling delay from the equation.

                  audio_lead_ms is a post-connection trim offset: positive values
                  delay video relative to audio.  Typical values:
                    0    — frames and audio bytes leave the host simultaneously
                    50   — adds ~50 ms video delay (compensates for I2S DMA latency
                           vs OLED render latency so both hit the user at the same time)

  OLED transport uses plain HTTP streaming (no micro-ROS agent required).
  The ESP32 connects to GET /lcd_out and reads 1024-byte frames continuously.

Topics
  Sub : /media/play  (std_msgs/String)  — absolute path to an .mp4 file
  Sub : /media/stop  (std_msgs/Empty)   — abort current playback

Parameters
  serve_port     int   8383  HTTP port for audio (same default as tts_node — don't run both)
  lcd_serve_port int   8384  HTTP port for OLED LCD stream
  threshold      int   127   greyscale → 1-bit cutoff (0-255)
  audio_lead_ms  int   0     ms to wait after audio ESP32 connects before first OLED frame
"""

import os
import queue
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

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
        filtered = np.empty_like(samples)
        z1 = z2 = 0.0
        for i, x in enumerate(samples):
            y      = b[0]*x + z1
            z1     = b[1]*x - a[1]*y + z2
            z2     = b[2]*x - a[2]*y
            filtered[i] = y

    return np.clip(filtered, -32768, 32767).astype("<i2").tobytes()

WIDTH = 128
HEIGHT = 64
PAGES = HEIGHT // 8   # 8
TARGET_FPS = 10       # OLED hard rate cap enforced by MCU bridge
SAMPLE_RATE = 16000   # Hz — matches tts_node default
BYTES_PER_SAMPLE = 2  # 16-bit signed LE


# ---------------------------------------------------------------------------
# Frame conversion helper
# ---------------------------------------------------------------------------

def _frame_to_oled(frame_data: bytes, threshold: int) -> bytes:
    """Convert raw 128×64 grayscale bytes to a 1024-byte SSD1306 framebuffer.

    SSD1306 page-major layout:
      8 pages × 128 columns = 1024 bytes
      Each byte = 8 vertical pixels; bit 0 = topmost row of the page.
    """
    img = np.frombuffer(frame_data, dtype=np.uint8).reshape(HEIGHT, WIDTH)
    bits = (img > threshold).astype(np.uint8)       # (64, 128) binary
    paged = bits.reshape(PAGES, 8, WIDTH)            # (8 pages, 8 rows/page, 128 cols)
    weights = (1 << np.arange(8, dtype=np.uint8))[:, np.newaxis]  # (8,1) bit masks
    fb = (paged * weights).sum(axis=1).astype(np.uint8)            # (8, 128)
    return bytes(fb.flatten())


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class Mp4PlayerNode(Node):
    def __init__(self):
        super().__init__("mp4_player")

        self.declare_parameter("serve_port", 8383)
        self.declare_parameter("lcd_serve_port", 8384)
        self.declare_parameter("threshold", 127)
        self.declare_parameter("audio_lead_ms", 0)
        self.declare_parameter("eq_bass_hz", 300.0)
        self.declare_parameter("eq_bass_db", 0.0)

        self._serve_port = (
            self.get_parameter("serve_port").get_parameter_value().integer_value
        )
        self._lcd_serve_port = (
            self.get_parameter("lcd_serve_port").get_parameter_value().integer_value
        )
        self._threshold = (
            self.get_parameter("threshold").get_parameter_value().integer_value
        )
        self._audio_lead_ms = (
            self.get_parameter("audio_lead_ms").get_parameter_value().integer_value
        )
        self._eq_bass_hz = (
            self.get_parameter("eq_bass_hz").get_parameter_value().double_value
        )
        self._eq_bass_db = (
            self.get_parameter("eq_bass_db").get_parameter_value().double_value
        )

        # Single-slot queues: one audio clip and one LCD frame at a time.
        self._audio_queue: queue.Queue[bytes] = queue.Queue(maxsize=1)
        self._lcd_queue: queue.Queue[bytes] = queue.Queue(maxsize=1)

        # Fired by the audio HTTP handler the instant it is about to push PCM
        # bytes to the ESP32.  _stream_video() blocks on this so video frames
        # never lead audio.
        self._audio_connected_event = threading.Event()

        self._stop_event = threading.Event()
        self._playing = False

        self._play_sub = self.create_subscription(
            String, "/media/play", self._on_play, 10
        )
        self._stop_sub = self.create_subscription(
            Empty, "/media/stop", self._on_stop, 10
        )

        self._start_http_server()
        self._start_lcd_server()
        self.get_logger().info(
            f"mp4_player ready — /media/play → "
            f":{self._serve_port}/audio_out + :{self._lcd_serve_port}/lcd_out"
        )

    # ---- ROS callbacks --------------------------------------------------------

    def _on_play(self, msg: String):
        filepath = msg.data.strip()
        if not filepath:
            return
        if self._playing:
            self.get_logger().warn("Already playing; send /media/stop first")
            return
        self.get_logger().info(f"Playing: {filepath}")
        threading.Thread(
            target=self._play_mp4, args=(filepath,), daemon=True
        ).start()

    def _on_stop(self, _msg: Empty):
        if self._playing:
            self.get_logger().info("Stop requested")
            self._stop_event.set()

    # ---- Playback pipeline ----------------------------------------------------

    def _play_mp4(self, filepath: str):
        if not os.path.isfile(filepath):
            self.get_logger().error(f"File not found: {filepath}")
            return

        self._playing = True
        self._stop_event.clear()
        self._audio_connected_event.clear()
        try:
            # 1. Extract and queue audio (sub-second for typical files).
            audio_ready = threading.Event()

            def _audio_worker():
                self._extract_and_queue_audio(filepath)
                audio_ready.set()

            threading.Thread(target=_audio_worker, daemon=True).start()

            # Wait for audio to be queued before releasing video (30 s safety cap).
            if not audio_ready.wait(timeout=30):
                self.get_logger().warn("Audio extraction timed out — continuing without audio")

            # 2. Stream video. _stream_video() blocks internally until the audio
            #    HTTP handler fires _audio_connected_event (ESP32 connected).
            if not self._stop_event.is_set():
                self._stream_video(filepath)
        finally:
            self._playing = False
            self._stop_event.clear()

    def _extract_and_queue_audio(self, filepath: str):
        """Decode the file's audio track to 16 kHz 16-bit mono PCM and enqueue."""
        cmd = [
            "ffmpeg", "-i", filepath,
            "-f", "s16le",
            "-ar", str(SAMPLE_RATE),
            "-ac", "1",
            "-loglevel", "quiet",
            "pipe:1",
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=120)
        except subprocess.TimeoutExpired:
            self.get_logger().error("Audio extraction timed out")
            return
        except FileNotFoundError:
            self.get_logger().error("ffmpeg not found — install it in the container")
            return

        pcm = result.stdout
        if not pcm:
            self.get_logger().warn("No audio track found in file")
            return

        pcm = _apply_lowshelf(pcm, SAMPLE_RATE, self._eq_bass_hz, self._eq_bass_db)
        duration_s = len(pcm) / (SAMPLE_RATE * BYTES_PER_SAMPLE)
        self.get_logger().info(
            f"Audio ready: {len(pcm):,} bytes PCM ({duration_s:.1f}s)"
        )
        try:
            self._audio_queue.put_nowait(pcm)
        except queue.Full:
            self.get_logger().warn("Audio queue full; skipping audio playback")

    def _stream_video(self, filepath: str):
        """Decode video to 128×64 grayscale and stream OLED frames at TARGET_FPS.

        Blocks until _audio_connected_event fires so the first video frame is
        sent only after the audio ESP32 has connected and PCM bytes are flowing.
        audio_lead_ms can trim the offset if audio and video arrive at slightly
        different times (positive = delay video further).
        """
        cmd = [
            "ffmpeg", "-i", filepath,
            "-f", "rawvideo",
            "-pix_fmt", "gray",
            "-vf", f"scale={WIDTH}:{HEIGHT}",
            "-r", str(TARGET_FPS),
            "-loglevel", "quiet",
            "pipe:1",
        ]
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
        except FileNotFoundError:
            self.get_logger().error("ffmpeg not found")
            return

        # Wait for audio ESP32 to connect before releasing the first frame.
        # Timeout of 60 s covers slow WiFi reconnects; on timeout we proceed
        # anyway so video isn't silently lost if there's no audio.
        if not self._audio_connected_event.wait(timeout=60.0):
            self.get_logger().warn(
                "Audio not connected after 60 s — starting video without sync"
            )
        elif self._audio_lead_ms > 0:
            time.sleep(self._audio_lead_ms / 1000.0)

        frame_size = WIDTH * HEIGHT        # 8192 bytes per frame
        interval = 1.0 / TARGET_FPS        # 0.1 s
        t_start = time.monotonic()
        frame_idx = 0

        try:
            while not self._stop_event.is_set():
                raw = proc.stdout.read(frame_size)
                if len(raw) < frame_size:
                    break  # EOF

                fb = _frame_to_oled(raw, self._threshold)
                try:
                    self._lcd_queue.put_nowait(fb)
                except queue.Full:
                    pass  # ESP32 hasn't consumed last frame yet — drop

                frame_idx += 1
                # Pace output to TARGET_FPS.
                target_t = t_start + frame_idx * interval
                delay = target_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        finally:
            proc.kill()
            proc.wait()
            self.get_logger().info(f"Video done: {frame_idx} frames streamed")

    # ---- HTTP server (mirrors tts_node pattern) -------------------------------

    def _start_http_server(self):
        node = self

        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/audio_out":
                    self._serve_audio()
                else:
                    self.send_response(404)
                    self.end_headers()

            def _serve_audio(self):
                """One PCM clip per connection.
                Returns 204 when idle so the ESP32 retries immediately, avoiding
                I2S DMA underrun (same behaviour as tts_node)."""
                try:
                    data = node._audio_queue.get(timeout=25)
                except queue.Empty:
                    self.send_response(204)
                    self.end_headers()
                    return

                duration_s = len(data) / (SAMPLE_RATE * BYTES_PER_SAMPLE)
                node.get_logger().info(
                    f"Streaming {len(data):,} bytes PCM ({duration_s:.1f}s) to ESP32"
                )
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                # Signal video pipeline: audio bytes are about to flow.
                # _stream_video() is waiting on this before sending first frame.
                node._audio_connected_event.set()
                try:
                    self.wfile.write(data)
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    node.get_logger().warn("Client disconnected during audio stream")

            def log_message(self, fmt, *args):
                node.get_logger().debug(fmt % args)

        server = HTTPServer(("0.0.0.0", self._serve_port), _Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"HTTP audio server listening on :{self._serve_port}")

    def _start_lcd_server(self):
        """Serve a persistent raw framebuffer stream on lcd_serve_port/lcd_out.

        The ESP32 connects once and reads 1024-byte frames continuously until
        disconnected.  Frames are sourced from _lcd_queue (maxsize=1, drop on
        full) so the display always shows the most recent frame.
        """
        node = self

        class _LcdHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/lcd_out":
                    self._serve_stream()
                else:
                    self.send_response(404)
                    self.end_headers()

            def _serve_stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.end_headers()
                node.get_logger().info("LCD stream connected")
                try:
                    while True:
                        try:
                            frame = node._lcd_queue.get(timeout=1.0)
                        except queue.Empty:
                            continue
                        self.wfile.write(frame)
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                node.get_logger().info("LCD stream disconnected")

            def log_message(self, fmt, *args):
                pass  # suppress per-request logs

        server = HTTPServer(("0.0.0.0", self._lcd_serve_port), _LcdHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"HTTP LCD server listening on :{self._lcd_serve_port}")


def main(args=None):
    rclpy.init(args=args)
    node = Mp4PlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
