"""ROS 2 node: /audio_tts_input → Fish Audio TTS → HTTP server for ESP32."""

import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import requests
except ImportError:
    requests = None


class TtsNode(Node):
    def __init__(self):
        super().__init__("tts_node")

        self.declare_parameter("fish_api_key", "")
        self.declare_parameter("fish_reference_id", "")
        self.declare_parameter("fish_model", "s2-pro")
        self.declare_parameter("serve_port", 8383)
        self.declare_parameter("sample_rate", 16000)

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

        if not self._api_key:
            self.get_logger().error(
                "No Fish Audio API key. Set FISH_API_KEY env var or fish_api_key param"
            )
        if requests is None:
            self.get_logger().error("python3-requests not installed")

        # Audio buffer: ESP32 long-polls GET /audio_out, blocks until data is
        # available, then receives the PCM and disconnects.
        self._audio_ready = threading.Event()
        self._audio_lock = threading.Lock()
        self._audio_data: bytes = b""

        self._start_http_server()

        self._sub = self.create_subscription(
            String, "/audio_tts_input", self._on_transcription, 10
        )
        self.get_logger().info(
            f"TTS node ready — listening on /audio_tts_input, "
            f"serving audio on :{self._serve_port}/audio_out"
        )

    # ---- ROS subscription callback -------------------------------------------

    def _on_transcription(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"TTS request: {text[:80]}")

        pcm = self._fish_tts(text)
        if pcm is None:
            return

        with self._audio_lock:
            self._audio_data = pcm
        self._audio_ready.set()

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
                """Persistent stream — stays open across multiple TTS events."""
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Transfer-Encoding", "chunked")
                self.end_headers()

                node.get_logger().info("Audio stream client connected")
                try:
                    while True:
                        node._audio_ready.wait()
                        with node._audio_lock:
                            data = node._audio_data
                            node._audio_data = b""
                            node._audio_ready.clear()
                        if data:
                            self.wfile.write(
                                f"{len(data):x}\r\n".encode() + data + b"\r\n"
                            )
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    node.get_logger().info("Audio stream client disconnected")

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
