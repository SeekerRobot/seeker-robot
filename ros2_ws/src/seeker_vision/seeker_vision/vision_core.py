import os
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool
from ultralytics import YOLO
from mcu_msgs.msg import DetectedObjectArray
from std_msgs.msg import Header
from seeker_vision.detection_utils import build_detection_array


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.detection_pub = self.create_publisher(DetectedObjectArray, '/vision/detections', 10)
        self.found_publisher = self.create_publisher(Bool, '/object_found', 10)

        self.declare_parameter('video_source', 'http://localhost:8080/stream')
        source = self.get_parameter('video_source').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        model_path = os.path.join(
            get_package_share_directory('seeker_vision'), 'model', 'yolo26n.pt'
        )
        self.model = YOLO(model_path)
        self.has_display = bool(os.environ.get('DISPLAY'))

        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 fps

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warning("Failed to grab frame from stream.")
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_optical_frame'
        detections_msg = build_detection_array(self.model, img, header)
        self.detection_pub.publish(detections_msg)

        found = Bool()
        found.data = len(detections_msg.detections) > 0
        self.found_publisher.publish(found)

        if self.has_display:
            cv2.imshow('Webcam', img)
            if cv2.waitKey(1) == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
