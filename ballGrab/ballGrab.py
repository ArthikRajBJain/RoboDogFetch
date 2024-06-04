import sys
import cv2
from ultralytics import YOLO

sys.path.append('../libraries/dynamixelAPI/')
from dynamixel import *
model = YOLO('../libraries/yoloDetection/ball.pt')

cap = cv2.VideoCapture(0)

setTorque(port,packet)
# positionHome(port,packet)
positionSuitable(port,packet)

start = 1

while cap.isOpened():
    success, frame = cap.read()
    if success:
        results = model(source=frame, conf=0.65, max_det=1, iou=0.5, imgsz=(256,160))
        annotated_frame = results[0].plot()
        box = results[0].boxes.xyxy.ravel().cpu().numpy()
        if(box != []):
            start = 0
            x1 = box[0]
            x2 = box[2]
            setPoint = ((640.0-(x2-x1))/2.0)
            error = x1 - setPoint
            print(error)
            if(abs(error) < 5):
                error = 0
            if(error !=0 ):
                positionZControl(port,packet,readPosition(port,packet,11) + int(-0.5*error))
        else:
            if(start == 1):
                continue
            presentPosition = readPosition(port,packet,11)
            if(x1 > setPoint):
                positionZControl(port,packet,presentPosition-50)
                if(presentPosition <= 950):
                    x1 = 0
                print('clockwise ' + 'x1 ' + str(x1) + ' setPoint ' + str(setPoint) + ' ' + str(presentPosition))
            else:
                positionZControl(port,packet,presentPosition+50)
                if(presentPosition >= 3150):
                    x1 = 640
                print('counter clockwise ' + 'x1 ' + str(x1) + ' setPoint ' + str(setPoint) + ' ' + str(presentPosition))

        cv2.imshow("YOLOv8 Inference", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break



cap.release()
cv2.destroyAllWindows()
