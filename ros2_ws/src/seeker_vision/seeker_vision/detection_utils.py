import cv2
from std_msgs.msg import Float32MultiArray
from mcu_msgs.msg import DetectedObject, DetectedObjectArray
from seeker_vision.constants import CLASS_NAMES


def run_detection(model, img, target_name, logger):
    """Run YOLO on img, annotate in-place. Returns (found_target, found_box).

    Legacy helper used by the HTTP-stream `vision_core` path. The Gazebo path
    uses `build_detection_array` below which publishes every detection.
    """
    found_target = False
    found_box = None

    for r in model(img, stream=True):
        for box in r.boxes:
            cls = int(box.cls[0])
            label = CLASS_NAMES[cls]

            if label == target_name:
                found_target = True
                found_box = box
                logger.debug(f"Target '{target_name}' found (conf={float(box.conf[0]):.2f})")

            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            cv2.putText(img, label, [x1, y1], cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    return found_target, found_box


def build_detection_msg(found_target, found_box, img_width):
    """Build Float32MultiArray detection message [area, cx, frame_width]."""
    msg = Float32MultiArray()
    if found_target and found_box is not None:
        x1, y1, x2, y2 = (int(v) for v in found_box.xyxy[0])
        msg.data = [
            float((x2 - x1) * (y2 - y1)),
            float((x1 + x2) / 2.0),
            float(img_width),
        ]
    else:
        msg.data = [0.0, 0.0, float(img_width)]
    return msg


def build_detection_array(model, img, header):
    """Run YOLO on img, annotate in-place, and return a DetectedObjectArray
    containing every detection. Consumers filter by class_name downstream.
    """
    msg = DetectedObjectArray()
    msg.header = header
    h, w = img.shape[:2]

    for r in model(img, stream=True):
        for box in r.boxes:
            cls = int(box.cls[0])
            label = CLASS_NAMES[cls]
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])

            d = DetectedObject()
            d.class_name = label
            d.confidence = float(box.conf[0])
            d.bbox_cx = (x1 + x2) / 2.0
            d.bbox_cy = (y1 + y2) / 2.0
            d.bbox_w = x2 - x1
            d.bbox_h = y2 - y1
            d.image_width = float(w)
            d.image_height = float(h)
            msg.detections.append(d)

            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 255), 3)
            cv2.putText(img, label, (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    return msg
