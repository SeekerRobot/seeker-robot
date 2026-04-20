import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool  # Added Bool import

from seeker_vision.constants import CLASS_NAMES

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
        target_object: str = "none" 
        response_phrase: str = "" 
else:
    Output = None

_INSTRUCTION = f"""
You are a robot named Hatsune. You receive commands between 'hey hatsune' and 'over'.
Your goal is to map the user's intent to one of the predefined tasks.

TASKS: {[a.value for a in RobotAction]}

SPECIAL RULE FOR 'find the object':
If you believe the command represents 'find the object', you MUST also identify 
which specific object the user wants from the following allowed list:
{CLASS_NAMES}

If the user mentions an object NOT in the list, pick the most logically similar 
item from the list (e.g., 'Coke' becomes 'bottle'). 

Respond using the provided JSON schema.
"""


# ---- State constants -------------------------------------------------------------

_IDLE = 0
_COLLECTING = 1
_CONFIRMING = 2


class CommandNode(Node):
    def __init__(self):
        super().__init__("command_node")

        self.declare_parameter("gemini_api_key", "")
        self.declare_parameter("gemini_model", "gemini-2.5-flash")
        self.declare_parameter("wake_word", "hey hatsune")
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
        
        self._client = genai.Client(api_key=api_key)

        # Publishers
        self._tts_pub = self.create_publisher(String, "/audio_tts_input", 10)
        self._cmd_pub = self.create_publisher(String, "/voice_command", 10)
        self.target_pub = self.create_publisher(String, '/target_object', 10)
        self.search_trigger_pub = self.create_publisher(Bool, '/search_trigger', 10)

        # Subscription
        self._sub = self.create_subscription(
            String, "/audio_transcription", self._on_transcription, 10
        )

        self._state = _IDLE
        self._command_parts: list[str] = []
        
        # Changed to store the whole Output object so we keep the 'target_object'
        self._pending_result: Output | None = None 
        
        self._timer: threading.Timer | None = None
        self._lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1)

        self.get_logger().info("command_node ready.")

    def _on_transcription(self, msg: String):
        text = msg.data.strip()
        text_lower = text.lower()

        with self._lock:
            # --- EMERGENCY STOP ---
            if "stop" in text_lower:
                self.get_logger().warn("Emergency Stop!")
                stop_msg = String()
                stop_msg.data = "stop"
                self._cmd_pub.publish(stop_msg)
                self._publish_tts("Stopping.")
                self._reset()
                return

            # --- STATE MACHINE ---
            if self._state == _IDLE:
                valid_wakes = [self._wake_word, "hey hat soon", "hey hot soup"]
                if any(wake in text_lower for wake in valid_wakes):
                    self._state = _COLLECTING
                    self._command_parts = [text]
                    self._restart_timeout()
                    if self._end_word in text_lower:
                        self._handle_command_complete()

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

    def _handle_command_complete(self):
        self._cancel_timeout()
        command_text = " ".join(self._command_parts)
        if self._end_word in command_text.lower():
            command_text = command_text.lower().split(self._end_word)[0] + self._end_word
        
        self.get_logger().info(f"Calling Gemini for: {command_text}")
        self._state = _CONFIRMING
        self._executor.submit(self._call_gemini, command_text)

    def _call_gemini(self, command_text: str):
        try:
            response = self._client.models.generate_content(
                model=self._model_name,
                contents=f"<user_command>\n{command_text}\n</user_command>",
                config=genai_types.GenerateContentConfig(
                    system_instruction=_INSTRUCTION,
                    response_mime_type="application/json",
                    response_schema=Output,
                ),
            )
            result = Output(**json.loads(response.text))
            
            with self._lock:
                self._pending_result = result
                self._restart_timeout()
            
            # Formulate the confirmation question
            action_desc = result.translated_command.value
            if result.translated_command == RobotAction.FIND:
                action_desc = f"finding the {result.target_object}"
            
            tts_text = f"I think you want me to {action_desc}. Is that right?"
            self._publish_tts(tts_text)
            
        except Exception as e:
            self.get_logger().error(f"Gemini error: {e}")
            self._publish_tts("I couldn't process that. Please try again.")
            with self._lock: self._reset()

    def _execute_command(self):
        with self._lock:
            if self._pending_result is None:
                self._reset()
                return
            
            res = self._pending_result
            action = res.translated_command
            self._reset()

        # 1. Handle "Find" specific logic
        if action == RobotAction.FIND:
            # Tell Vision what to look for
            target_msg = String()
            target_msg.data = res.target_object
            self.target_pub.publish(target_msg)

            # Kick off the search state
            search_msg = Bool()
            search_msg.data = True
            self.search_trigger_pub.publish(search_msg)
            
            self.get_logger().info(f"Searching for: {res.target_object}")

        # 2. General command publication
        cmd_msg = String()
        cmd_msg.data = action.value
        self._cmd_pub.publish(cmd_msg)

        # 3. Use Gemini's custom response phrase if it provided one
        feedback = res.response_phrase if res.response_phrase else f"Executing {action.value}."
        self._publish_tts(feedback)

    def _abort_command(self):
        self._reset()
        self._publish_tts("Okay, I'll ignore that.")

    def _on_timeout(self):
        with self._lock:
            if self._state != _IDLE:
                self._reset()
                self._publish_tts("Listening timed out.")

    def _reset(self):
        self._cancel_timeout()
        self._state = _IDLE
        self._command_parts = []
        self._pending_result = None

    def _restart_timeout(self):
        self._cancel_timeout()
        self._timer = threading.Timer(self._timeout_s, self._on_timeout)
        self._timer.daemon = True
        self._timer.start()

    def _cancel_timeout(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def _publish_tts(self, text: str):
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)

    def destroy_node(self):
        self._executor.shutdown(wait=False)
        with self._lock: self._cancel_timeout()
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