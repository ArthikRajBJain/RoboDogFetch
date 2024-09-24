import sys
import cv2
from ultralytics import YOLO
import time

sys.path.append('../libraries/dynamixelAPI/')
sys.path.append('../libraries/inverseKinematics/')
from dynamixel import *
from ik import *
model = YOLO('../libraries/yoloDetection/ball.pt')

def distanceMap(boxX):
    out = 0
    if(boxX >= 157 and boxX <= 350):
        out = 0.0028*(boxX**2) - 1.9118*boxX + 480.5943
    if(boxX < 157 and boxX >= 80):
        out = -0.00067437*(boxX**3) + 0.27581*(boxX**2) - 38.937*boxX + 2173.5
    return out

while 1:
    cap = cv2.VideoCapture(2)
    setTorque(port,packet)
    slowDown(port,packet,1000)
    positionSuitable(port,packet)

    start = 1
    sameCount = 0
    x2x1_prev = 0
    x1_prev = 0

    while cap.isOpened():
        success, frame = cap.read()
        if success:
            results = model(source=frame, conf=0.65, max_det=1, iou=0.5, imgsz=(256,160), verbose=False)
            annotated_frame = results[0].plot()
            box = results[0].boxes.xyxy.ravel().cpu().numpy()
            if(box.size > 0):
                start = 0
                x1 = box[0]
                x2 = box[2]
                setPoint = ((640.0-(x2-x1))/2.0)
                error = x1 - setPoint
                if(abs(error) < 5):
                    error = 0
                if(error !=0 ):
                    positionZControl(port,packet,readPosition(port,packet,11) + int(-0.5*error))
                if(abs((x2-x1) - x2x1_prev) < abs((x2-x1)*0.005)) and (abs((x1) - x1_prev) < abs((x1)*0.005)):
                    sameCount = sameCount + 1
                else:
                    x2x1_prev = x2 - x1
                    x1_prev = x1
                if(sameCount == 100):
                    positionOpenArm(port,packet)
                    distanceCorrection = int(distanceMap(x2-x1)/10.0)
                    if(distanceCorrection > 30):
                        distanceCorrection = distanceCorrection + 3
                    i12, j13, k14 = inverseKinematics(distanceCorrection)
                    slowDown(port,packet,2000)
                    writePosition(port,packet,-1,i12,j13,k14,-1)
                    time.sleep(3)
                    slowDown(port,packet,0)
                    positionCloseArm(port,packet)
                    time.sleep(1)
                    presentCurrent = readCurrent(port,packet,15)
                    # print("presentCurrent : ")
                    # print(presentCurrent)
                    if(presentCurrent < 20):
                        sameCount = 0
                        slowDown(port,packet,1000)
                        positionSuitable(port,packet)
                        continue
                    time.sleep(2)
                    positionReturnBall(port,packet)
                    slowDown(port,packet,1000)
                    time.sleep(7)
                    positionOpenArm(port,packet)
                    positionHome(port,packet)
                    time.sleep(3)
                    releaseTorque(port,packet)
                    break
            else:
                if(start == 1):
                    continue
                presentPosition = readPosition(port,packet,11)
                if(x1 > setPoint):
                    positionZControl(port,packet,presentPosition-50)
                    if(presentPosition <= 950):
                        x1 = 0
                else:
                    positionZControl(port,packet,presentPosition+50)
                    if(presentPosition >= 3150):
                        x1 = 640

            cv2.imshow("YOLOv8 Inference", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            break


    cap.release()
    cv2.destroyAllWindows()
