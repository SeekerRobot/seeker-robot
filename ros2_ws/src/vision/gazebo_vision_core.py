import cv2
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

classNames = [
    "person", "bicycle", "car", "motorbike", "aeroplane",
    "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird",
    "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut",
    "cake", "chair", "sofa", "pottedplant", "bed",
    "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # --- GAZEBO: image comes from a ROS2 topic via cv_bridge ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',   #! Change to Gazebo camera topic
            self.image_callback,
            10
        )

        # Publisher: sends True when the teddy bear is found
        self.publisher_ = self.create_publisher(Bool, '/object_found', 10)

        self.model = YOLO("yolo26n.pt")
        self.target_name = "teddy bear"

    def image_callback(self, msg):
        # Convert ROS2 Image message to OpenCV frame
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(img, stream=True)
        found_target = False

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                label = classNames[cls]

                if label == self.target_name:
                    found_target = True
                    print("DESIRED TARGET FOUND -------------------------------------------------")

                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                # confidence
                confidence = math.ceil((box.conf[0] * 100)) / 100
                print("Confidence --->", confidence)

                # class name
                cls = int(box.cls[0])
                print("Class name -->", classNames[cls], "\n")

                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2
                cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

        # Publish found status to ROS2
        found_msg = Bool()
        found_msg.data = found_target
        self.publisher_.publish(found_msg)

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