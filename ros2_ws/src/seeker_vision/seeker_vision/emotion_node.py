import os
import cv2
import rclpy
from deepface import DeepFace
from rclpy.node import Node
from std_msgs.msg import String


class EmotionDetectionNode(Node):
    def __init__(self):
        super().__init__('emotion_detection_node')

        self.emotion_pub = self.create_publisher(String, '/emotion_detail', 10)

        self.declare_parameter('video_source', 'http://localhost:8080/stream')
        source = self.get_parameter('video_source').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(source)

        # Haar Cascade for fast face localisation before DeepFace
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        self.has_display = bool(os.environ.get('DISPLAY'))
        self.last_emotion = None

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 fps — DeepFace is slow

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warning("Failed to grab frame from stream.")
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )

        dominant_emotion = "No face detected"

        if len(faces) > 0:
            for (x, y, w, h) in faces:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

            try:
                analysis = DeepFace.analyze(
                    img, actions=['emotion'], enforce_detection=False
                )
                dominant_emotion = analysis[0]['dominant_emotion']
                cv2.putText(img, dominant_emotion, (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            except Exception as e:
                self.get_logger().warning(f"DeepFace error: {e}")

        if dominant_emotion != self.last_emotion:
            msg = String()
            msg.data = dominant_emotion
            self.emotion_pub.publish(msg)
            self.last_emotion = dominant_emotion

        if self.has_display:
            cv2.imshow('Emotion', img)
            if cv2.waitKey(1) == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EmotionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
