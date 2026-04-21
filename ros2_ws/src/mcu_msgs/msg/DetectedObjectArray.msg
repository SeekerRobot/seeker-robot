# One frame's worth of YOLO detections. Published per input image.

std_msgs/Header header          # stamp copied from source image; frame_id = camera_optical_frame
DetectedObject[] detections
