from ultralytics import YOLO

# Load a model
model = YOLO("yolo26n.pt") 

# Train the model
model.train(data="coco.yaml", epochs=100, imgsz=640)