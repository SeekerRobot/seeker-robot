import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from pydantic import BaseModel
except ImportError:
    BaseModel = None

try:
    from google import genai
    from google.genai import types as genai_types
except ImportError:
    genai = None
    genai_types = None


# ---- Pydantic schema -------------------------------------------------------------

class RobotAction(str, Enum):
    FORWARD = "move forward"
    BACKWARD = "move backward"
    LEFT = "move left"
    RIGHT = "move right"
    SPIN = "spin"
    DANCE = "dance"
    FIND = "find the object"
    NONE = "NONE OF THE ABOVE"


if BaseModel is not None:
    class Output(BaseModel):
        translated_command: RobotAction
else:
    Output = None

_INSTRUCTION = """\
You are a robot named billy. You are given a command by a user that may
represent a task from a predefined set of tasks. The command will take place
between the key phrases 'hey billy' [COMMAND] 'over'. Unfortunately, the
given command will not always be exactly any task. Additionally, the command
may be interrupted by other words picked up from other people. Therefore, you
must guess which task the user intended for you to command. If you do not
believe that the command accurately represents any task, opt not to pick any
task. The predefined set of tasks is: ['move forward', 'move backward', 'move
left', 'move right', 'spin', 'dance', 'find the object', 'NONE OF THE ABOVE'].
You may be lenient because your result will be verified by a human.
"""


# ---- State constants -------------------------------------------------------------

_IDLE = 0
_COLLECTING = 1
_CONFIRMING = 2


class CommandNode(Node):
    def __init__(self):
        super().__init__("command_node")

        self.declare_parameter("gemini_api_key", "")
        # Corrected model name to 1.5-flash
        self.declare_parameter("gemini_model", "gemini-2.5-flash")
        self.declare_parameter("wake_word", "hey billy")
        self.declare_parameter("end_word", "over")
        self.declare_parameter("idle_timeout_seconds", 30.0)

        api_key = (
            self.get_parameter("gemini_api_key").get_parameter_value().string_value
            or os.environ.get("GEMINI_API_KEY", "")
        )
        self._model_name = (
            self.get_parameter("gemini_model").get_parameter_value().string_value
        )
        self._wake_word = (
            self.get_parameter("wake_word").get_parameter_value().string_value.lower()
        )
        self._end_word = (
            self.get_parameter("end_word").get_parameter_value().string_value.lower()
        )
        self._timeout_s = (
            self.get_parameter("idle_timeout_seconds").get_parameter_value().double_value
        )

        if genai is None:
            self.get_logger().error("google-genai not installed — command_node disabled")
            return
        if not api_key:
            self.get_logger().warn(
                "GEMINI_API_KEY not set — Gemini calls will fail. "
                "Set the gemini_api_key parameter or GEMINI_API_KEY env var."
            )
        self._client = genai.Client(api_key=api_key)

        self._tts_pub = self.create_publisher(String, "/audio_tts_input", 10)
        self._cmd_pub = self.create_publisher(String, "/voice_command", 10)
        self._sub = self.create_subscription(
            String, "/audio_transcription", self._on_transcription, 10
        )

        self._state = _IDLE
        self._command_parts: list[str] = []
        self._pending_action: RobotAction | None = None
        self._timer: threading.Timer | None = None
        self._lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)

        self.get_logger().info(
            f"command_node ready — wake_word='{self._wake_word}', "
            f"end_word='{self._end_word}', timeout={self._timeout_s}s"
        )

    def _on_transcription(self, msg: String):
        # Verbose debug logging
        text = msg.data.strip()
        print(f"DEBUG: Node received: {text}")
        text_lower = text.lower()

        with self._lock:
            # --- EMERGENCY STOP OVERRIDE ---
            if "stop" in text_lower:
                self.get_logger().warn("OVERRIDE HEARD: Halting immediately!")
                stop_msg = String()
                stop_msg.data = "stop"
                self._cmd_pub.publish(stop_msg)
                self._publish_tts("Stopping.")
                self._reset()
                return

            # --- STATE MACHINE LOGIC ---
            if self._state == _IDLE:
                if self._wake_word in text_lower:
                    self.get_logger().info("Wake word detected — collecting command...")
                    self._state = _COLLECTING
                    self._command_parts = [text]
                    self._restart_timeout()
                    
                    # Check if the sentence is already complete
                    if self._end_word in text_lower:
                        self._handle_command_complete()
                    return

            elif self._state == _COLLECTING:
                self._command_parts.append(text)
                self._restart_timeout()
                if self._end_word in text_lower:
                    self._handle_command_complete()

            elif self._state == _CONFIRMING:
                if "yes" in text_lower:
                    self._execute_command()
                elif "no" in text_lower:
                    self._abort_command()

    # ---- State transitions -------------------------------------------------------

    def _handle_command_complete(self):
        self._cancel_timeout()
        command_text = " ".join(self._command_parts)
        if self._end_word in command_text.lower():
            # Keep text before the end word
            command_text = (
                command_text.lower().split(self._end_word)[0] + self._end_word
            )
        self.get_logger().info(f"Command collected: '{command_text}' — calling Gemini…")
        self._state = _CONFIRMING
        self._command_parts = []
        self._executor.submit(self._call_gemini, command_text)

    def _call_gemini(self, command_text: str):
        try:
            injected = f"<user_command>\n{command_text}\n</user_command>"
            response = self._client.models.generate_content(
                model=self._model_name,
                contents=injected,
                config=genai_types.GenerateContentConfig(
                    system_instruction=_INSTRUCTION,
                    response_mime_type="application/json",
                    response_schema=Output,
                ),
            )
            result = Output(**json.loads(response.text))
            with self._lock:
                self._pending_action = result.translated_command
                self._restart_timeout()
            
            tts_text = (
                f"I received '{command_text}' and translated it to "
                f"'{result.translated_command.value}'. "
                f"Say yes or no, then {self._end_word}."
            )
            self._publish_tts(tts_text)
            self.get_logger().info(f"Gemini result: {result.translated_command}")
        except Exception as e:
            self.get_logger().error(f"Gemini error: {e}")
            self._publish_tts("Sorry, I could not process that command.")
            with self._lock:
                self._reset()

    def _execute_command(self):
        if self._pending_action is None:
            self._reset()
            return
        action = self._pending_action
        self._reset()

        cmd_msg = String()
        cmd_msg.data = action.value
        self._cmd_pub.publish(cmd_msg)

        self._publish_tts(f"Proceeding with '{action.value}'.")
        self.get_logger().info(f"Executing command: {action.value}")

    def _abort_command(self):
        self._reset()
        self._publish_tts("Abandoning command.")
        self.get_logger().info("Command aborted by user.")

    def _on_timeout(self):
        with self._lock:
            if self._state != _IDLE:
                self.get_logger().warn(f"Timeout — returning to idle.")
                self._reset()
        self._publish_tts("Timed out. I am listening for a new command.")

    def _reset(self):
        self._cancel_timeout()
        self._state = _IDLE
        self._command_parts = []
        self._pending_action = None

    # ---- Timeout helpers ---------------------------------------------------------

    def _restart_timeout(self):
        self._cancel_timeout()
        self._timer = threading.Timer(self._timeout_s, self._on_timeout)
        self._timer.daemon = True
        self._timer.start()

    def _cancel_timeout(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    # ---- Publishing helpers ------------------------------------------------------

    def _publish_tts(self, text: str):
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)

    # ---- Lifecycle ---------------------------------------------------------------

    def destroy_node(self):
        self._executor.shutdown(wait=False)
        with self._lock:
            self._cancel_timeout()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()