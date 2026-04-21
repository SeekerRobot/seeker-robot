# Single YOLO detection in an image frame.
# Bounding-box coordinates are in image pixels.

string  class_name    # COCO class label (e.g. "teddy bear", "sports ball")
float32 confidence    # YOLO confidence in [0, 1]
float32 bbox_cx       # bounding-box centre x (pixels)
float32 bbox_cy       # bounding-box centre y (pixels)
float32 bbox_w        # bounding-box width  (pixels)
float32 bbox_h        # bounding-box height (pixels)
float32 image_width   # source image width  (pixels) — consumers compute bearing from bbox_cx / image_width
float32 image_height  # source image height (pixels)
