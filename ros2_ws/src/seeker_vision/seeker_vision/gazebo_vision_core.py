import os
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from mcu_msgs.msg import DetectedObjectArray
from seeker_vision.detection_utils import build_detection_array


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        self.detection_pub = self.create_publisher(DetectedObjectArray, '/vision/detections', 10)
        self.found_publisher = self.create_publisher(Bool, '/object_found', 10)

        model_path = os.path.join(
            get_package_share_directory('seeker_vision'), 'model', 'yolo26n.pt'
        )
        self.model = YOLO(model_path)
        self.has_display = bool(os.environ.get('DISPLAY'))

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        header = msg.header
        if not header.frame_id:
            header.frame_id = 'camera_optical_frame'
        detections_msg = build_detection_array(self.model, img, header)
        self.detection_pub.publish(detections_msg)

        found = Bool()
        found.data = len(detections_msg.detections) > 0
        self.found_publisher.publish(found)

        if self.has_display:
            cv2.imshow('Gazebo Vision', img)
            if cv2.waitKey(1) == ord('q'):
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
