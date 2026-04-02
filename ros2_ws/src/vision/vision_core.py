import cv2
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from ultralytics import YOLO
from deepface import DeepFace
from deepface import DeepFace
from std_msgs.msg import String
import math
import numpy as np

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

        # Publisher: sends True when the teddy bear is found
        self.found_publisher = self.create_publisher(Bool, '/object_found', 10)

        # Publisher: sends [bbox_area, centroid_x, img_width] 
        self.detection_pub = self.create_publisher(Float32MultiArray, '/detection_detail', 10)

        self.emotion_publisher = self.create_publisher(String, '/emotion_detail', 10)

        # --- REAL LIFE: image comes from the ESP32 stream URL ---
        self.cap = cv2.VideoCapture("http://192.168.8.200/stream")  #! Change to ESP32 stream URL
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        self.model = YOLO("yolo26n.pt")
        self.target_name = "teddy bear"

        # Timer drives the detection loop 
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 fps

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to grab frame from stream.")
            return

        results = self.model(img, stream=True)
        found_target = False
        found_box = None  # store the teddy bear box to publish detection data


        ##############################################
        # Emotion Detection Starts here
        ##############################################

        #Convert frame to grayscale (Haar Cascade works better with grayscale images)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # detect faces in the frame
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        
        # this will draw rectangles around detected faces as shown
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Only run DeepFace if at least one face found
        if len(faces) > 0:
            try:
                emotion_analysis = DeepFace.analyze(img, actions=['emotion'], enforce_detection=False)
                dominant_emotion = emotion_analysis[0]['dominant_emotion']
                cv2.putText(img, dominant_emotion, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            except ValueError:
                pass

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls = int(box.cls[0])
                label = classNames[cls]

                if label == self.target_name:
                    found_target = True
                    found_box = box
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

        # Publish simple found flag to /object_found
        found_msg = Bool()
        found_msg.data = found_target
        self.found_publisher.publish(found_msg)

        # Publish [bbox_area, centroid_x, img_width] to /detection_detail
        detection_msg = Float32MultiArray()

        if found_target and found_box is not None:
            x1, y1, x2, y2 = found_box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # find the details of the box
            bbox_area  = float((x2 - x1) * (y2 - y1))
            centroid_x = float((x1 + x2) / 2.0)
            img_width  = float(img.shape[1])
            detection_msg.data = [bbox_area, centroid_x, img_width]
        else:
            detection_msg.data = [0.0, 0.0, float(img.shape[1])]
       
        self.detection_pub.publish(detection_msg)

        # Publish emotion data to /emotion_detail
        emotion = String()
        if len(faces) > 0:
            emotion.data = dominant_emotion
        else:
            emotion.data = "No face detected"

        self.emotion_publisher.publish(emotion)

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