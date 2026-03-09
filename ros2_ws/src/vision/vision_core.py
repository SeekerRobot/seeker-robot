import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO 

class RobotVisionNode(Node):
    def __init__(self):
        super().__init__('robot_vision_node')
        
        # load trained model
        self.model = YOLO("best.pt") 
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.process_image, 10) #! Change image topic if needed
        self.marker_pub = self.create_publisher(String, 'map_updates', 10)
        self.bridge = CvBridge()
        
        self.target_name = "spoon" #! Change to target type as desired
                                   # This target type is a classifier type in the yolo data set

    def process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # stream=True makes it run faster for live video
        results = self.model(frame, stream=True, conf=0.6)

        found_target = False
        found_wrong_object = False

        for r in results:
            for box in r.boxes:
                # get the name of the object detected
                class_id = int(box.cls[0])
                label = self.model.names[class_id]

                if label == self.target_name:
                    found_target = True
                else:
                    found_wrong_object = True

        # Mapping
        if found_target:
            self.get_logger().info("GOAL FOUND!")
        elif found_wrong_object:
            # mark the spot on the map to avoid this area 
            self.get_logger().info("Wrong object detected. Marking map.")
            status_msg = String()
            status_msg.data = "mark_as_obstacle"
            self.marker_pub.publish(status_msg)

        # Drawing the boxes on the screen so you can see what the robot sees
        annotated_frame = next(results).plot()
        cv2.imshow("YOLO Robot View", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RobotVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()