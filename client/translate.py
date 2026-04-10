from enum import Enum
import threading
from pydantic import BaseModel
import json
from google.genai import types
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from google import genai
import re
    
# mm yummy instructuosnsf 
class RobotAction(str, Enum):
    FORWARD = 'move forward'
    BACKWARD = 'move backward'
    LEFT = 'move left'
    RIGHT = 'move right'
    SPIN = 'spin'
    DANCE = 'dance'
    FIND = 'find the object'
    NONE = 'NONE OF THE ABOVE'
    
class Output(BaseModel):
    translated_command: RobotAction
    
# global instr to be accessed by all 
INSTR = (
    "You are a robot named Hatsune Miku. You are given a command by "
    "a user that may represent a task from a predefined set of tasks. "
    "Unfortunately, the given command will not always be exactly any task. "
    "Therefore, you must guess which task the user intended to command. "
    "If you do not believe that the command accurately represents any task, "
    "opt not to pick any task. The predefined set of tasks is: "
    "['move forward', 'move backward', 'move left', 'move right', "
    "'spin', 'dance', 'find the object', 'NONE OF THE ABOVE']"
)

class TranslateNode(Node):

    def __init__(self):
        super().__init__('translate_node')

        # subscribe to transcribe node to get the messages
        self.subscription = self.create_subscription(
            String,
            'transcription',          
            self.transcription_callback,  
            10
        )

        # i also want to publish :D
        self.robot_command_publisher = self.create_publisher(String, 'robot_command', 10)

        self.client = genai.Client() # ai slop ai ai ai 

        # flag and temp storage for command
        self.listening = False
        self.command = []
        self.command_lock = threading.Lock() # THREADDINNNGGG LOCKSSS RACE CONDISTIONSSS
    
    # every time a msg is received, this callback will be called
    def transcription_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Received transcription: {text}")

        if not self.listening:
            if 'hey hatsune' in text.lower():
                self.listening = True
                with self.command_lock:
                    self.command = []   # reset any leftover command from before
                self.get_logger().info("Listening for command...")
                return

        if self.listening:
            if re.search(r'\bover\b', text.lower()): # re is searching for the word "over" with \b (boundary) as opposed to just "over" in text
                self.listening = False
                self.translate_and_publish()  
            else:
                with self.command_lock:  # preventing race conditions 
                    self.command.append(text)


    def translate_and_publish(self):
        with self.command_lock:
            if not self.command:
                    return
            curr_command = self.command.copy() # take current command and then clear
            self.command = [] # clear the commands
            
        injected_prompt: str = f"""
            <user_command>
        {' '.join(curr_command)}
            </user_command>
        """
        
        try:
            response = self.client.models.generate_content(
                model = 'gemini-2.5-flash',
                contents = injected_prompt,
                config = types.GenerateContentConfig(
                    system_instruction = INSTR,
                    response_mime_type = 'application/json',
                    response_schema = Output
                )
            )
            
            full_response_obj = Output(**json.loads(response.text))
        
            translated_command = full_response_obj.translated_command
            self.get_logger().info(f"Robot command: {translated_command.value}")

            # Publish thy command now
            out_msg = String()
            out_msg.data = translated_command.value
            self.robot_command_publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Translation error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TranslateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
    