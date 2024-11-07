import socket
import cv2
import time
import sys
import struct

# Define the host and port to connect to
HOST = '192.168.76.28'
PORT = 65432

ARRAY_SIZE = 10

sys.path.append('../libraries/dynamixelAPI/')
sys.path.append('../libraries/inverseKinematics/')
from dynamixel import *
from ik import *

# Open a video capture (use 0 for default webcam or provide a file path)
cap = cv2.VideoCapture(0)
setTorque(port,packet)
slowDown(port,packet,1000)
positionSuitable(port,packet)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((HOST, PORT))

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            break

        # Encode the frame as JPEG
        _, buffer = cv2.imencode('.jpg', frame)
        frame_data = buffer.tobytes()

        # Send the length of the frame first
        frame_length = len(frame_data)
        client_socket.sendall(frame_length.to_bytes(4, byteorder='big'))

        # Send the frame data
        client_socket.sendall(frame_data)

        # Receive the error variable from the server
        array_data = client_socket.recv(ARRAY_SIZE * 4)
        response_array = struct.unpack(f'>{ARRAY_SIZE}i', array_data)

        if(response_array[0] != 0):
            if(response_array[0] == 1):
                positionZControl(port,packet,response_array[1])
            if(response_array[0] == 2):
                positionOpenArm(port,packet)
                i12, j13, k14 = inverseKinematics(response_array[2])
                slowDown(port,packet,2000)
                writePosition(port,packet,-1,i12,j13,k14,-1)
                time.sleep(3)
                slowDown(port,packet,0)
                positionCloseArm(port,packet)
                time.sleep(1)
                slowDown(port,packet,2000)
                presentCurrent = readCurrent(port,packet,15)
                if(presentCurrent < 20):
                    sameCount = 0
                    slowDown(port,packet,1000)
                    positionSuitable(port,packet)
                    continue
                time.sleep(2)
                slowDown(port,packet,2000)
                positionReturnBall(port,packet)
                time.sleep(7)
                positionOpenArm(port,packet)
                positionHome(port,packet)
                time.sleep(3)
                releaseTorque(port,packet)


cap.release()
cv2.destroyAllWindows()
