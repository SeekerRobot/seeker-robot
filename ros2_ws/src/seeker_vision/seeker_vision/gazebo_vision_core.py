import os
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String  # Added String
from ultralytics import YOLO
from mcu_msgs.msg import HexapodCmd
from seeker_vision.detection_utils import run_detection, build_detection_msg
from seeker_vision.constants import CLASS_NAMES # Ensure this is where your list lives

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.found_publisher = self.create_publisher(Bool, '/object_found', 10)
        self.detection_pub = self.create_publisher(Float32MultiArray, '/detection_detail', 10)
        self.pub = self.create_publisher(HexapodCmd, '/mcu/hexapod_cmd', 10)

        # --- NEW: Subscribe to the target object command ---
        self.target_sub = self.create_subscription(
            String, 
            '/target_object', 
            self.target_callback, 
            10
        )

        self.declare_parameter('video_source', 'http://localhost:8080/stream')
        source = self.get_parameter('video_source').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(source)
        
        # Performance tip: If your laptop is weak, ensure these match the stream resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        model_path = os.path.join(
            get_package_share_directory('seeker_vision'), 'model', 'yolo26n.pt'
        )
        self.model = YOLO(model_path)
        
        # Default target
        self.target_name = "sports ball" 
        self.has_display = bool(os.environ.get('DISPLAY'))

        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 fps

    def target_callback(self, msg):
        """Updates the hunting target when the Brain sends a new name."""
        new_target = msg.data.lower().strip()
        if new_target in CLASS_NAMES:
            self.target_name = new_target
            self.get_logger().info(f"Target updated! Now searching for: {self.target_name}")
        else:
            self.get_logger().warn(f"Received invalid target: {new_target}. Staying on {self.target_name}")

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warning("Failed to grab frame from stream.")
            return

        # detection_utils.run_detection now uses the dynamic self.target_name
        found_target, found_box = run_detection(self.model, img, self.target_name, self.get_logger())

        found_msg = Bool()
        found_msg.data = found_target
        self.found_publisher.publish(found_msg)

        self.detection_pub.publish(build_detection_msg(found_target, found_box, img.shape[1]))

        # Keep your dance logic if you want Hatsune to celebrate finding it!
        if found_target:
            msg = HexapodCmd()
            msg.mode = HexapodCmd.MODE_DANCE
            self.pub.publish(msg)

        if self.has_display:
            cv2.imshow('Hatsune Vision', img)
            if cv2.waitKey(1) == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()

# ... (main remains the same)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
