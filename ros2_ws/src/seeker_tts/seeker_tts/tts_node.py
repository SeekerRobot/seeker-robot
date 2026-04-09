"""ROS 2 node: /audio_transcription → Fish Audio TTS → ESP32 speaker."""

import os

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
        self.declare_parameter("speaker_url", "http://192.168.1.100:82/speak")
        self.declare_parameter("sample_rate", 16000)

        self._api_key = (
            self.get_parameter("fish_api_key").get_parameter_value().string_value
            or os.environ.get("FISH_API_KEY", "")
        )
        self._ref_id = (
            self.get_parameter("fish_reference_id")
            .get_parameter_value()
            .string_value
        )
        self._model = (
            self.get_parameter("fish_model").get_parameter_value().string_value
        )
        self._speaker_url = (
            self.get_parameter("speaker_url").get_parameter_value().string_value
        )
        self._sample_rate = (
            self.get_parameter("sample_rate")
            .get_parameter_value()
            .integer_value
        )

        if not self._api_key:
            self.get_logger().error(
                "No Fish Audio API key. Set param fish_api_key or env FISH_API_KEY"
            )

        if requests is None:
            self.get_logger().error("python3-requests not installed")

        self._sub = self.create_subscription(
            String, "/audio_transcription", self._on_transcription, 10
        )
        self.get_logger().info(
            f"TTS node ready — listening on /audio_transcription, "
            f"speaker at {self._speaker_url}"
        )

    def _on_transcription(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"TTS request: {text[:80]}")

        pcm = self._fish_tts(text)
        if pcm is None:
            return

        self._send_to_speaker(pcm)

    def _fish_tts(self, text: str) -> bytes | None:
        """Call Fish Audio TTS and return raw 16-bit PCM bytes."""
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

        chunks = []
        for chunk in resp.iter_content(chunk_size=4096):
            chunks.append(chunk)

        pcm = b"".join(chunks)
        duration_s = len(pcm) / (self._sample_rate * 2)
        self.get_logger().info(
            f"Got {len(pcm)} bytes PCM ({duration_s:.1f}s audio)"
        )
        return pcm

    def _send_to_speaker(self, pcm: bytes):
        """POST raw PCM to the ESP32 speaker HTTP endpoint."""
        try:
            resp = requests.post(
                self._speaker_url,
                data=pcm,
                headers={"Content-Type": "application/octet-stream"},
                timeout=30 + len(pcm) / (self._sample_rate * 2),
            )
            resp.raise_for_status()
            self.get_logger().info("Audio sent to speaker")
        except requests.RequestException as e:
            self.get_logger().error(f"Speaker POST failed: {e}")


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
