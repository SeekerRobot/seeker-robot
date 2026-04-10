import threading
import requests
import numpy as np
from faster_whisper import WhisperModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import timer

ESP32_IP = "http://192.168.8.50:81"
URL = f"{ESP32_IP}/audio"
SAMPLE_RATE = 16000
WINDOW_SECONDS = 2
BUFFER_THRESHOLD = SAMPLE_RATE * WINDOW_SECONDS

# Make a ROS2 node yippee yippee
class TranscribeNode(Node):
    def __init__(self):
        super().__init__('transcribe_node')

        # create thy publisher 
        self.publisher_ = self.create_publisher(String, 'transcription', 10)

        self.get_logger().info("Loading Whisper model...")
        self.model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
        self.get_logger().info("Whisper model loaded. Connecting to stream...")

        # perform threading to make sure that the transcription process does not block the ROS2 node
        self.stream_thread = threading.Thread(
            target=self.transcribe, 
            daemon=True
        )
        self.stream_thread.start()

    def transcribe(self):
        while rclpy.ok(): # keep running even if connection down (it will keep retrying every 5 seconds)
            audio_buffer = np.array([], dtype = np.float32)
            
            self.get_logger().info(f"Connecting to {URL}")

            try:
                with requests.get(URL, stream = True, timeout = 10) as r:
                    r.raise_for_status()
                    self.get_logger().info("Connected! Transcription started. Speak into the mic.")
                    
                    for chunk in r.iter_content(chunk_size = 2048):
                        if not chunk:
                            break
                        
                        pcm_data = np.frombuffer(chunk, dtype = np.int16)
                        audio_float = pcm_data.astype(np.float32) / 32768.0
                        
                        audio_buffer = np.append(audio_buffer, audio_float)
                        
                        if len(audio_buffer) >= BUFFER_THRESHOLD:
                            segments, _ = self.model.transcribe(audio_buffer, beam_size = 5)
                            
                            for segment in segments:
                                if segment.text.strip():
                                    # i want to send the text in a ROS2 message
                                    msg = String()
                                    msg.data = segment.text.strip()

                                    # NOW PUBLISH THE MSG yayay
                                    self.publisher_.publish(msg)
                                    self.get_logger().info(f"Published: {msg.data}")
                                    
                            audio_buffer = np.array([], dtype = np.float32)
            
            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Connection error: {e}")
                timer.sleep(5)  # wait before retrying

def main(args=None):
    rclpy.init(args=args)
    node = TranscribeNode() # create the node
    rclpy.spin(node) # run the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
