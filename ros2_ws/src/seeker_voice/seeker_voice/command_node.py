import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool

from mcu_msgs.action import SeekObject
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
You are a robot named Hatsune (Hatsune Miku persona). You receive commands
between 'hey hatsune' and 'over'. Your goal is to map the user's intent to
one of the predefined tasks.

TASKS: {[a.value for a in RobotAction]}

SPECIAL RULE FOR 'find the object':
If you believe the command represents 'find the object', you MUST also identify
which specific object the user wants from the following allowed list:
{CLASS_NAMES}

If the user mentions an object NOT in the list, pick the most logically similar
item from the list (e.g., 'Coke' becomes 'bottle').

RESPONSE_PHRASE RULE (ALWAYS fill this field):
Produce a short, energetic Hatsune Miku-style acknowledgment — cute, upbeat,
playful. Under 12 words. Vary it every time so it never feels repetitive.
Do NOT include a question or the word "right" — the caller appends its own
confirmation question. Reference the action (and target object for 'find').
Examples:
  - "Okay, zooming forward~!"
  - "Hehe, spinning around for you!"
  - "On it! Hunting down the bottle~ ♪"
  - "Yay, let's dance together!"
  - "Mmm, I don't quite get that one..."  (for NONE)

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

        api_key = self.get_parameter("gemini_api_key").get_parameter_value().string_value
        if not api_key:
            api_key = os.environ.get("GEMINI_API_KEY", "")
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
            self.get_logger().warn("google-genai not installed — command_node will only use heuristic fallback.")
            self._client = None
        elif api_key:
            self._client = genai.Client(api_key=api_key)
        else:
            self._client = None
            self.get_logger().warn("No GEMINI_API_KEY provided. Gemini features will be disabled, using heuristics only.")

        # Publishers
        self._tts_pub = self.create_publisher(String, "/audio_tts_input", 10)
        self._cmd_pub = self.create_publisher(String, "/voice_command", 10)
        self.target_pub = self.create_publisher(String, '/target_object', 10)
        self.search_trigger_pub = self.create_publisher(Bool, '/search_trigger', 10)

        # Action Client (New Seeker Interface)
        self._action_client = ActionClient(self, SeekObject, 'seek_object')

        # Subscription
        self._sub = self.create_subscription(
            String, "/audio_transcription", self._on_transcription, 10
        )

        self._state = _IDLE
        self._command_parts: list[str] = []
        
        # Changed to store the whole Output object so we keep the 'target_object'
        self._pending_result: Output | None = None 
        
        self._timer: threading.Timer | None = None
        self._lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=1)

        self.get_logger().info("command_node ready.")

    def _on_transcription(self, msg: String):
        text = msg.data.strip()
        text_lower = text.lower()

        with self._lock:
            self.get_logger().info(f"Received transcription: '{text}' (Current State: {self._state})")
            
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
                valid_wakes = [
                    self._wake_word,
                    "hey hatsune",
                    "hey hat soon",
                    "hey hot soup",
                    "hey hot sun",
                    "hey hatsoon",
                    "hey hatsun",
                    "hey hatsuna",
                    "hey hatsume",
                    "hey hatsuni",
                    "hey hot sune",
                    "hey hat sune",
                    "hey hatsuney",
                    "hatsune",
                ]
                if any(wake in text_lower for wake in valid_wakes):
                    self.get_logger().info("Wake word detected! Switching to COLLECTING.")
                    self._state = _COLLECTING
                    self._command_parts = [text]
                    self._restart_timeout()
                    if self._end_word in text_lower:
                        self._handle_command_complete()

            elif self._state == _COLLECTING:
                self.get_logger().info("Collecting command parts...")
                self._command_parts.append(text)
                self._restart_timeout()
                if self._end_word in text_lower:
                    self.get_logger().info("End word detected! Handling command complete.")
                    self._handle_command_complete()

            elif self._state == _CONFIRMING:
                self.get_logger().info(f"Waiting for confirmation. Text: '{text_lower}'")
                # More robust "yes" detection: look for 'yes', 'yeah', 'correct', 'do it', etc.
                positive_responses = ["yes", "yeah", "correct", "do it", "sure", "yep", "ok"]
                negative_responses = ["no", "nope", "wrong", "cancel", "stop", "don't"]
                
                if any(pos in text_lower for pos in positive_responses):
                    self.get_logger().info("Confirmation received (YES). Executing...")
                    self._execute_command()
                elif any(neg in text_lower for neg in negative_responses):
                    self.get_logger().info("Confirmation received (NO). Aborting.")
                    self._abort_command()
                else:
                    self.get_logger().info("Did not understand confirmation. Still waiting...")

    def _handle_command_complete(self):
        self._cancel_timeout()
        command_text = " ".join(self._command_parts)
        if self._end_word in command_text.lower():
            command_text = command_text.lower().split(self._end_word)[0] + self._end_word
        
        self.get_logger().info(f"Command complete: {command_text}")
        self._state = _CONFIRMING
        
        if self._client:
            # Dispatch to background thread so we don't block the ROS callback queue
            self._executor.submit(self._call_gemini, command_text)
        else:
            self.get_logger().warn("Gemini client missing. Skipping API call, jumping to fallback.")
            self._executor.submit(self._call_fallback, command_text)

    def _call_gemini(self, command_text: str):
        import traceback
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
                self.get_logger().info(f"Gemini parsed: action={result.translated_command}, target={result.target_object}")
            
            # Formulate the confirmation question — prefer Gemini's dynamic
            # Miku-style response_phrase; fall back to a plain description.
            if result.response_phrase:
                tts_text = f"{result.response_phrase} Is that right?"
            else:
                action_desc = result.translated_command.value
                if result.translated_command == RobotAction.FIND:
                    action_desc = f"finding the {result.target_object}"
                tts_text = f"I think you want me to {action_desc}. Is that right?"
            self._publish_tts(tts_text)
            
        except Exception as e:
            self.get_logger().error(f"Gemini error: {e}")
            self.get_logger().error(traceback.format_exc())
            self._call_fallback(command_text)

    def _call_fallback(self, command_text: str):
        # --- HEURISTIC FALLBACK ---
        # This allows the robot to function even if the API is offline or the key is exhausted.
        self.get_logger().warn("Attempting heuristic fallback...")
        text = command_text.lower()
        
        fallback_result = None
        if "forward" in text:
            fallback_result = Output(translated_command=RobotAction.FORWARD, response_phrase="Moving forward.")
        elif "backward" in text:
            fallback_result = Output(translated_command=RobotAction.BACKWARD, response_phrase="Moving backward.")
        elif "left" in text:
            fallback_result = Output(translated_command=RobotAction.LEFT, response_phrase="Turning left.")
        elif "right" in text:
            fallback_result = Output(translated_command=RobotAction.RIGHT, response_phrase="Turning right.")
        elif "spin" in text:
            fallback_result = Output(translated_command=RobotAction.SPIN, response_phrase="Spinning around.")
        elif "dance" in text:
            fallback_result = Output(translated_command=RobotAction.DANCE, response_phrase="Let's dance!")
        elif "find" in text or "search" in text or "look" in text:
            # Try to extract target object from the allowed list
            target = "sports ball" # default to something in CLASS_NAMES
            for obj in CLASS_NAMES:
                if obj.lower() in text:
                    target = obj
                    break
            fallback_result = Output(translated_command=RobotAction.FIND, target_object=target, response_phrase=f"Searching for the {target}.")

        if fallback_result:
            self.get_logger().info(f"Heuristic matched: {fallback_result.translated_command}")
            with self._lock:
                self._pending_result = fallback_result
                self._restart_timeout()
            
            action_desc = fallback_result.translated_command.value
            if fallback_result.translated_command == RobotAction.FIND:
                action_desc = f"finding the {fallback_result.target_object}"
            
            # We add a note to the TTS so the user knows it's using the fallback
            prefix = "Gemini is offline, but " if self._client else ""
            self._publish_tts(f"{prefix}I think you want me to {action_desc}. Is that right?")
            return
        
        self._publish_tts("I couldn't process that. Please try again.")
        with self._lock: self._reset()

    def _execute_command(self):
        with self._lock:
            if self._pending_result is None:
                self.get_logger().error("Execute called but pending_result is None!")
                self._reset()
                return
            
            res = self._pending_result
            action = res.translated_command
            self.get_logger().info(f"Executing action: {action.value}")
            self._reset()

        # 1. Handle "Find" specific logic
        if action == RobotAction.FIND:
            self.get_logger().info(f"Initiating search for: {res.target_object}")

            # Update legacy target topic for monitoring
            target_msg = String()
            target_msg.data = res.target_object
            self.target_pub.publish(target_msg)

            # Request autonomous search via Action Server
            if self._action_client.wait_for_server(timeout_sec=1.0):
                goal_msg = SeekObject.Goal()
                goal_msg.class_name = res.target_object
                self._action_client.send_goal_async(goal_msg).add_done_callback(self._goal_response_callback)
            else:
                self.get_logger().error("SeekObject Action Server not available!")
                self._publish_tts("I can't start the search because the navigation system is offline.")

            # Trigger legacy search state
            search_msg = Bool()
            search_msg.data = True
            self.search_trigger_pub.publish(search_msg)

        self._finish_up_command(action, res)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Search goal rejected by server.")
            return
        self.get_logger().info("Search goal accepted. Awaiting result...")
        goal_handle.get_result_async().add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"SUCCESS: {result.message}")
            self._publish_tts("Goal complete! I've reached the target.")
        else:
            self.get_logger().warn(f"FAILED: {result.message}")
            self._publish_tts("The search mission was unsuccessful.")

    # 2. General command publication
    def _finish_up_command(self, action, res):
        cmd_msg = String()
        cmd_msg.data = action.value
        self.get_logger().info(f"Publishing to /voice_command: {cmd_msg.data}")
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