#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceTerminal(Node):
    def __init__(self):
        super().__init__('voice_terminal')
        self.publisher = self.create_publisher(String, '/audio_transcription', 10)
        self.get_logger().info("Voice Terminal Ready. Type your command below.")

    def run(self):
        while rclpy.ok():
            try:
                text = input("🗣️ > ")
                if not text.strip(): continue
                msg = String()
                msg.data = text
                self.publisher.publish(msg)
            except EOFError:
                break
            except KeyboardInterrupt:
                break

def main():
    rclpy.init()
    node = VoiceTerminal()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
