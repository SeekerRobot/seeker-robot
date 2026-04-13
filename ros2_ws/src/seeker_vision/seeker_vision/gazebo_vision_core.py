import os
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from seeker_vision.detection_utils import run_detection, build_detection_msg


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

        self.found_publisher = self.create_publisher(Bool, '/object_found', 10)
        self.detection_pub = self.create_publisher(Float32MultiArray, '/detection_detail', 10)

        model_path = os.path.join(
            get_package_share_directory('seeker_vision'), 'model', 'yolo26n.pt'
        )
        self.model = YOLO(model_path)
        self.target_name = "teddy bear"
        self.has_display = bool(os.environ.get('DISPLAY'))

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        found_target, found_box = run_detection(self.model, img, self.target_name, self.get_logger())

        found_msg = Bool()
        found_msg.data = found_target
        self.found_publisher.publish(found_msg)

        self.detection_pub.publish(build_detection_msg(found_target, found_box, img.shape[1]))

        if self.has_display:
            cv2.imshow('Webcam', img)
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
