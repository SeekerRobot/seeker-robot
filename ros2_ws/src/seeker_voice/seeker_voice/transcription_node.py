"""ROS 2 node: audio ingestion (ESP32 HTTP stream or local mic) → Whisper → /audio_transcription.

TTS deadlock prevention
-----------------------
SpeakerSubsystem::fetchAndPlay() calls mic->pause() → MicSubsystem::stopServer() →
httpd_stop() on the first audio chunk it receives. httpd_stop() blocks until all
active HTTP sessions close. If this node holds an open streaming connection to the
ESP32 mic server, httpd_stop() waits forever → speaker never writes to I2S → audio
appears to play only after this node exits.

Fix: subscribe to /audio_tts_input. When TTS fires, immediately drop the ESP32
connection so httpd_stop() can complete. Reconnect after tts_pause_seconds (enough
time for TTS to play and the mic to be resumed by the speaker).

Optional passthrough server (passthrough_port != 0):
  GET http://localhost:<passthrough_port>/audio  — raw 16-bit LE mono PCM
  GET http://localhost:<passthrough_port>/stream — browser HTML player

Listen with:
  curl -s http://localhost:8386/audio | aplay -r 16000 -f S16_LE -c 1
"""

import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import requests
except ImportError:
    requests = None

try:
    from faster_whisper import WhisperModel
except ImportError:
    WhisperModel = None

try:
    import sounddevice as sd
except (ImportError, OSError):
    sd = None

# HTML player — same Web Audio API approach as test_raw_mic/mic_web_server.cpp
_PLAYER_HTML = """\
<!DOCTYPE html>
<html>
<head><title>Mic Passthrough</title></head>
<body>
<button id="btn">Start Listening</button>
<p id="status"></p>
<script>
const SAMPLE_RATE = {sample_rate};
document.getElementById('btn').onclick = async () => {{
  document.getElementById('status').textContent = 'Connecting...';
  const ctx = new AudioContext({{ sampleRate: SAMPLE_RATE }});
  const res = await fetch('/audio');
  const reader = res.body.getReader();
  let nextTime = ctx.currentTime + 0.1;
  document.getElementById('status').textContent = 'Streaming';
  while (true) {{
    const {{ done, value }} = await reader.read();
    if (done) break;
    const pcm = new Int16Array(value.buffer, value.byteOffset, value.byteLength / 2);
    const f32 = new Float32Array(pcm.length);
    for (let i = 0; i < pcm.length; i++) f32[i] = pcm[i] / 32768.0;
    const buf = ctx.createBuffer(1, f32.length, SAMPLE_RATE);
    buf.copyToChannel(f32, 0);
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.connect(ctx.destination);
    const t = Math.max(ctx.currentTime + 0.05, nextTime);
    src.start(t);
    nextTime = t + buf.duration;
  }}
}};
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Audio broadcaster — fans out raw PCM bytes to all connected HTTP clients
# ---------------------------------------------------------------------------

class _AudioBroadcaster:
    def __init__(self):
        self._clients: list[queue.Queue] = []
        self._lock = threading.Lock()

    def add_client(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=100)
        with self._lock:
            self._clients.append(q)
        return q

    def remove_client(self, q: queue.Queue) -> None:
        with self._lock:
            try:
                self._clients.remove(q)
            except ValueError:
                pass

    def broadcast(self, data: bytes) -> None:
        with self._lock:
            for q in self._clients:
                try:
                    q.put_nowait(data)
                except queue.Full:
                    pass  # drop if client is too slow — never block audio loop


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

class _PassthroughHandler(BaseHTTPRequestHandler):
    broadcaster: _AudioBroadcaster
    sample_rate: int = 16000

    def do_GET(self):
        if self.path == "/audio":
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = self.broadcaster.add_client()
            try:
                while True:
                    try:
                        chunk = q.get(timeout=5.0)
                    except queue.Empty:
                        continue
                    if chunk is None:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                self.broadcaster.remove_client(q)

        elif self.path in ("/", "/stream"):
            body = _PLAYER_HTML.format(sample_rate=self.sample_rate).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        pass  # suppress per-request logging


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class TranscriptionNode(Node):
    def __init__(self):
        super().__init__("transcription_node")

        self.declare_parameter("audio_source", "esp32")
        self.declare_parameter("esp32_ip", "192.168.8.50")
        self.declare_parameter("esp32_port", 81)
        self.declare_parameter("esp32_path", "/audio")
        self.declare_parameter("whisper_model", "base")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("window_seconds", 2.0)
        self.declare_parameter("passthrough_port", 8386)
        self.declare_parameter("min_audio_rms", 0.01)
        # How long to stay disconnected from the ESP32 mic after TTS fires.
        # Must be long enough to cover: Fish Audio API latency + full audio
        # duration + speaker reconnect cycle (kRetryIntervalMs = 3 s).
        # The mic is also paused by SpeakerSubsystem during playback, so the
        # ESP32 server is unavailable until playback finishes regardless.
        self.declare_parameter("tts_pause_seconds", 12.0)

        self._audio_source = (
            self.get_parameter("audio_source").get_parameter_value().string_value
        )
        self._esp32_ip = (
            self.get_parameter("esp32_ip").get_parameter_value().string_value
        )
        self._esp32_port = (
            self.get_parameter("esp32_port").get_parameter_value().integer_value
        )
        self._esp32_path = (
            self.get_parameter("esp32_path").get_parameter_value().string_value
        )
        whisper_model_name = (
            self.get_parameter("whisper_model").get_parameter_value().string_value
        )
        self._sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        window_seconds = (
            self.get_parameter("window_seconds").get_parameter_value().double_value
        )
        self._buffer_threshold = int(self._sample_rate * window_seconds)
        self._passthrough_port = (
            self.get_parameter("passthrough_port").get_parameter_value().integer_value
        )
        self._min_rms = (
            self.get_parameter("min_audio_rms").get_parameter_value().double_value
        )
        self._tts_pause_seconds = (
            self.get_parameter("tts_pause_seconds").get_parameter_value().double_value
        )

        self._pub = self.create_publisher(String, "/audio_transcription", 10)

        # Event that signals the audio loop to disconnect. Set on TTS trigger,
        # cleared after tts_pause_seconds by a one-shot threading.Timer.
        self._tts_active = threading.Event()
        self._tts_timer: threading.Timer | None = None
        self._tts_lock = threading.Lock()

        self._tts_sub = self.create_subscription(
            String, "/audio_tts_input", self._on_tts_input, 10
        )

        # Start passthrough server before Whisper load so it's available immediately
        self._broadcaster = _AudioBroadcaster()
        if self._passthrough_port != 0:
            self._start_passthrough_server()

        if WhisperModel is None:
            self.get_logger().error("faster-whisper not installed — cannot transcribe")
            return

        self.get_logger().info(f"Loading Whisper model '{whisper_model_name}'…")
        self._model = WhisperModel(whisper_model_name, device="cpu", compute_type="int8")
        self.get_logger().info("Whisper model loaded.")

        self._thread = threading.Thread(target=self._audio_loop, daemon=True)
        self._thread.start()

    # ---- TTS coordination -------------------------------------------------------

    def _on_tts_input(self, msg: String):
        """Drop the ESP32 mic connection immediately so SpeakerSubsystem can
        call mic->pause() → httpd_stop() without blocking."""
        with self._tts_lock:
            if self._tts_timer is not None:
                self._tts_timer.cancel()
            self._tts_active.set()
            self._tts_timer = threading.Timer(
                self._tts_pause_seconds, self._resume_after_tts
            )
            self._tts_timer.daemon = True
            self._tts_timer.start()
        self.get_logger().info(
            f"TTS triggered — dropping ESP32 mic connection for "
            f"{self._tts_pause_seconds:.0f}s to unblock SpeakerSubsystem"
        )

    def _resume_after_tts(self):
        self._tts_active.clear()
        self.get_logger().info("TTS pause expired — reconnecting to ESP32 mic stream")

    # ---- Passthrough server ------------------------------------------------------

    def _start_passthrough_server(self):
        _PassthroughHandler.broadcaster = self._broadcaster
        _PassthroughHandler.sample_rate = self._sample_rate

        server = HTTPServer(("0.0.0.0", self._passthrough_port), _PassthroughHandler)
        t = threading.Thread(target=server.serve_forever, daemon=True)
        t.start()
        self.get_logger().info(
            f"Audio passthrough on :{self._passthrough_port}  "
            f"curl -s http://localhost:{self._passthrough_port}/audio | "
            f"aplay -r {self._sample_rate} -f S16_LE -c 1  |  "
            f"browser: http://localhost:{self._passthrough_port}/stream"
        )

    # ---- Audio ingestion ---------------------------------------------------------

    def _audio_loop(self):
        if self._audio_source == "esp32":
            self._esp32_loop()
        elif self._audio_source == "local":
            self._local_loop()
        else:
            self.get_logger().error(
                f"Unknown audio_source '{self._audio_source}'. Use 'esp32' or 'local'."
            )

    def _esp32_loop(self):
        if requests is None:
            self.get_logger().error("requests not installed — cannot connect to ESP32")
            return

        url = f"http://{self._esp32_ip}:{self._esp32_port}{self._esp32_path}"
        self.get_logger().info(f"Connecting to ESP32 audio stream at {url}")

        while rclpy.ok():
            # Block here (not just sleep) while TTS is playing — the ESP32 mic
            # server is paused by SpeakerSubsystem and won't accept connections.
            if self._tts_active.is_set():
                self._tts_active.wait()
                self.get_logger().info("Reconnecting to ESP32 audio stream")

            try:
                with requests.get(url, stream=True, timeout=10) as r:
                    r.raise_for_status()
                    self.get_logger().info("ESP32 audio stream connected.")
                    buf = np.array([], dtype=np.float32)

                    for chunk in r.iter_content(chunk_size=2048):
                        # TTS fired — exit cleanly so httpd_stop() unblocks.
                        if self._tts_active.is_set():
                            self.get_logger().info(
                                "Disconnecting from ESP32 mic (TTS active)"
                            )
                            break

                        if not chunk:
                            break

                        # Broadcast raw int16 bytes to passthrough clients.
                        self._broadcaster.broadcast(chunk)

                        # ESP32: raw 16-bit signed LE mono PCM. Normalize for Whisper.
                        pcm = (
                            np.frombuffer(chunk, dtype=np.int16)
                            .astype(np.float32) / 32768.0
                        )
                        buf = np.append(buf, pcm)

                        if len(buf) >= self._buffer_threshold:
                            self._transcribe(buf.copy())
                            buf = np.array([], dtype=np.float32)

            except Exception as e:
                if not self._tts_active.is_set():
                    self.get_logger().warn(
                        f"ESP32 audio connection error: {e} — retrying in 2 s"
                    )
                time.sleep(2.0)

    def _local_loop(self):
        if sd is None:
            self.get_logger().error("sounddevice not installed — cannot use local mic")
            return

        self.get_logger().info("Starting local microphone capture.")
        buf: list = []
        lock = threading.Lock()

        def _callback(indata, frames, t, status):
            if self._tts_active.is_set():
                return
            raw = (indata[:, 0] * 32767).astype(np.int16)
            self._broadcaster.broadcast(raw.tobytes())
            with lock:
                buf.append(indata[:, 0].copy())

        with sd.InputStream(
            samplerate=self._sample_rate,
            channels=1,
            dtype="float32",
            callback=_callback,
        ):
            accumulated = np.array([], dtype=np.float32)
            while rclpy.ok():
                time.sleep(0.05)
                if self._tts_active.is_set():
                    accumulated = np.array([], dtype=np.float32)
                    continue
                with lock:
                    if buf:
                        accumulated = np.append(accumulated, np.concatenate(buf))
                        buf.clear()
                if len(accumulated) >= self._buffer_threshold:
                    self._transcribe(accumulated.copy())
                    accumulated = np.array([], dtype=np.float32)

    # ---- Transcription -----------------------------------------------------------

    def _transcribe(self, audio: np.ndarray):
        if self._min_rms > 0.0:
            rms = float(np.sqrt(np.mean(audio ** 2)))
            if rms < self._min_rms:
                self.get_logger().debug(f"Skipping low-energy audio (RMS={rms:.4f})")
                return

        try:
            segments, _ = self._model.transcribe(
                audio, 
                beam_size=5,
                # initial_prompt="Hey Hatsune. Hatsune Miku."
            )
            
            for segment in segments:
                text = segment.text.strip()
                if text:
                    msg = String()
                    msg.data = text
                    self._pub.publish(msg)
                    self.get_logger().info(f"Transcribed: {text}")

        except Exception as e:
            self.get_logger().error(f"Whisper transcription error: {e}")
            
    # ---- Lifecycle ---------------------------------------------------------------

    def destroy_node(self):
        with self._tts_lock:
            if self._tts_timer is not None:
                self._tts_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TranscriptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
