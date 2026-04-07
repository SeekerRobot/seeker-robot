# Code to use your device's camera to practice opencv and obj detection
# This is not for the robot, but for reference!!

import cv2
from ultralytics import YOLO 
import math
import numpy as np
from deepface import DeepFace


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


cap = cv2.VideoCapture(0)

#Let's load the pre-trained Haar Cascade Classifier for face detection
# face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

cap.set(3, 640)
cap.set(4, 480)

model = YOLO("yolo26n.pt")

while True:
    success, img = cap.read()
    results = model(img, stream=True)

    ##############################################
    # Emotion Detection Starts here
    # but we are not really using it so nevermind. commented!!
    ##############################################

    # #Convert frame to grayscale (Haar Cascade works better with grayscale images)
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # # detect faces in the frame
    # faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    # # this will draw rectangles around detected faces as shown
    # for (x, y, w, h) in faces:
    #     cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

    # # Only run DeepFace if at least one face found
    # if len(faces) > 0:
    #     try:
    #         emotion_analysis = DeepFace.analyze(img, actions=['emotion'], enforce_detection=False)
    #         dominant_emotion = emotion_analysis[0]['dominant_emotion']
    #         cv2.putText(img, dominant_emotion, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
    #     except ValueError:
    #         pass


    ##############################################
    # Object Detection Starts here
    ##############################################

    found_target = False
    target_name = "teddy bear"

    # coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:

            # get the name of the object detected
            cls = int(box.cls[0])
            label = classNames[cls]
            

            if label == target_name:
                found_target = True
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

            # put box in cam
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0]*100))/100
            print("Confidence --->",confidence)

            # class name
            cls = int(box.cls[0])
            print("Class name -->", classNames[cls],"\n")

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

    cv2.imshow('Webcam', img)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()