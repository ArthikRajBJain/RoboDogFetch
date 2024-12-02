import sys
import cv2
from ultralytics import YOLO
import time

# Custom fine tuned model to detect the ball
model = YOLO('../libraries/yoloDetection/ball.pt')
# Custom High Level Control library for the Robot
sys.path.append('../libraries/unitreeAPI/')
from unitreeAPI import *

# Custom library written to control the Arm
sys.path.append('../libraries/dynamixelAPI/')
# Custom library with pre-calculated dictionary to read the Inverse Kinematic Angles
sys.path.append('../libraries/inverseKinematics/')
from ik import *

# Distance Estimation using the Curve Fit Equation when the Arm is attached to the Robot
def distanceMap(boxX):
    out = 0.0000027821*(boxX**4) - 0.0016156*(boxX**3) + 0.35248*(boxX**2) - 35.944*boxX + 1699.2
    return out

framenumber = 0

while not(framenumber > 450): # Track the Tennis ball at the setpoint distance with the robot for 450 frames
    cap = cv2.VideoCapture(2) # Caturing a image frame from the external camera
    standUp(cmd,udp) # Set torque on the Arm
    while cap.isOpened():
        success, frame = cap.read()
        if success:
            framenumber = framenumber + 1
            print(framenumber)
            results = model(source=frame, conf=0.65, max_det=1, iou=0.5, imgsz=(640,480), verbose=False)
            annotated_frame = results[0].plot()
            box = results[0].boxes.xyxy.ravel().cpu().numpy()
            if(box.size > 0):
                start = 0
                x1 = box[0] # x1 co-ordinate of the bounding box
                x2 = box[2] # x2 co-ordinate of the bounding box
                error = 120 - (x2-x1) # Translational Error
                setPoint = ((640.0-(x2-x1))/2.0) # Setpoint - The desired location where the bounding box should be, so the ball is the centre of the frame
                rot_error = x1 - setPoint # Rotational Error
                orientDirectionless(cmd,udp,error*0.004,rot_error*-0.003) # Move the robot with Translational and Rotational motion praportionally to the respective errors
            cv2.imshow("YOLOv8 Inference", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            break
        if(framenumber > 450):
            break


    cap.release()
    cv2.destroyAllWindows()

sleepFlat(cmd,udp)
time.sleep(4)
lowPowerSleep(cmd,udp)

from dynamixel import *

while 1: # Pick the Tennis Ball with the Arm
    cap = cv2.VideoCapture(2) # Caturing a image frame from the external camera
    setTorque(port,packet) # Set torque on the Arm
    slowDown(port,packet,1000) # Slow down the movement of Arm
    positionSuitable(port,packet) # Place the Arm in the suitable position to manipulate

    start = 1
    sameCount = 0
    x2x1_prev = 0
    x1_prev = 0

    while cap.isOpened():
        success, frame = cap.read()
        if success:
            results = model(source=frame, conf=0.65, max_det=1, iou=0.5, imgsz=(256,160), verbose=False)
            annotated_frame = results[0].plot()
            box = results[0].boxes.xyxy.ravel().cpu().numpy() # Get the bounding box co-ordinates
            if(box.size > 0):
                start = 0
                x1 = box[0] # x1 co-ordinate of the bounding box
                x2 = box[2] # x2 co-ordinate of the bounding box
                setPoint = ((640.0-(x2-x1))/2.0) # Setpoint - The desired location where the bounding box should be, so the ball is the centre of the frame
                error = x1 - setPoint # Error to correct to centre the ball to the centre of the frame
                if(abs(error) < 5): # Set a deadzone to avoid oscillation when error is less enough
                    error = 0
                if(error !=0 ):
                    positionZControl(port,packet,readPosition(port,packet,11) + int(-0.5*error)) # The Z-Axis of the robot is corrected to center the ball (Proportional Controller)
                if(abs((x2-x1) - x2x1_prev) < abs((x2-x1)*0.005)) and (abs((x1) - x1_prev) < abs((x1)*0.005)): # Check if the ball is in the centre and increment
                    sameCount = sameCount + 1
                else:
                    x2x1_prev = x2 - x1
                    x1_prev = x1
                if(sameCount == 100): # Checking if the ball was in the center of the frame for 100 consecutive frames
                    positionOpenArm(port,packet) # Open the gripper
                    distanceCorrection = int(distanceMap(x2-x1)/10.0) # Convert Distance from millimeter to centimeter
                    print('boxX = ', end="")
                    print(x2-x1)
                    print('distance = ', end="")
                    print(distanceCorrection)
                    if(distanceCorrection >= 30): # Calibration to account for mechanical irregularity in the Arm and the Robot
                        distanceCorrection = distanceCorrection + 4
                    elif(distanceCorrection > 20):
                        distanceCorrection = distanceCorrection + 2
                    i12, j13, k14 = inverseKinematics(distanceCorrection) # Read the joint angle values from the pre-calculated IK values
                    slowDown(port,packet,2000)
                    writePosition(port,packet,-1,i12,j13,k14,-1) # Move the Arm to grab the ball
                    time.sleep(3)
                    slowDown(port,packet,0)
                    positionCloseArm(port,packet)
                    time.sleep(1)
                    slowDown(port,packet,2000)
                    presentCurrent = readCurrent(port,packet,15) # Read the current consumption of the gripper motor, to check if the ball is gripped or not
                    # print("presentCurrent : ")
                    # print(presentCurrent)
                    if(presentCurrent < 20): # The current consumed by gripper motor is less if the motor is not gripper anything
                        sameCount = 0
                        slowDown(port,packet,1000)
                        positionSuitable(port,packet)
                        continue # Restart the Algorithm because the gripping was not successful
                    time.sleep(2)
                    slowDown(port,packet,2000)
                    positionReturnBall(port,packet) # The gripping was successful hence returing the ball position
                    time.sleep(7)
                    positionOpenArm(port,packet) # Release the ball
                    positionHome(port,packet)
                    time.sleep(3)
                    releaseTorque(port,packet)
                    break
            else:
                if(start == 1):
                    continue
                presentPosition = readPosition(port,packet,11)
                if(x1 > setPoint): # Algorithm to scan the ball in all the positions reachable if the ball is out of the frame
                    positionZControl(port,packet,presentPosition-50)
                    if(presentPosition <= 950):
                        x1 = 0
                else:
                    positionZControl(port,packet,presentPosition+50)
                    if(presentPosition >= 3150):
                        x1 = 640

            cv2.imshow("YOLOv8 Inference", annotated_frame) # Show the bounding box on the ball and live correction to center the ball on screen

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            break


    cap.release()
    cv2.destroyAllWindows()
