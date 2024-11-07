import socket
import cv2
import numpy as np
import sys
from ultralytics import YOLO
import time
import struct

model = YOLO('../libraries/yoloDetection/ball.pt')

# Define the host and port to listen on
HOST = '192.168.76.28'
PORT = 65432

ARRAY_SIZE = 3
response_array = [0, 2040, 0]
# 0 - Change - 0,1,2
# 1 - Z
# 2 - distanceCorrection

def distanceMap(boxX):
    out = 0.0000027821*(boxX**4) - 0.0016156*(boxX**3) + 0.35248*(boxX**2) - 35.944*boxX + 1690.2
    return out

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print("Server is listening for connections...")
    conn, addr = server_socket.accept()
    with conn:
        print(f"Connected by {addr}")
        start = 1
        sameCount = 0
        x2x1_prev = 0
        x1_prev = 0
        while True:
            # Receive frame length first
            data = conn.recv(4)
            if not data:
                break
            frame_length = int.from_bytes(data, byteorder='big')

            # Receive the actual frame data
            frame_data = b''
            while len(frame_data) < frame_length:
                packet = conn.recv(frame_length - len(frame_data))
                if not packet:
                    break
                frame_data += packet

            # Decode the frame
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            if frame is not None:
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
                        response_array[0] = 0
                    if(error !=0 ):
                        response_array[1] = int(-0.5*error)
                        response_array[0] = 1
                    if(abs((x2-x1) - x2x1_prev) < abs((x2-x1)*0.005)) and (abs((x1) - x1_prev) < abs((x1)*0.005)):
                        sameCount = sameCount + 1
                    else:
                        x2x1_prev = x2 - x1
                        x1_prev = x1
                    if(sameCount == 100):
                        response_array[0] = 2
                        distanceCorrection = int(distanceMap(x2-x1)/10.0)
                        print('boxX = ', end="")
                        print(x2-x1)
                        print('distance = ', end="")
                        print(distanceCorrection)
                        if(distanceCorrection >= 30):
                            distanceCorrection = distanceCorrection + 4
                        elif(distanceCorrection > 20):
                            distanceCorrection = distanceCorrection + 2
                        response_array[2] = distanceCorrection
                        break
                else:
                    if(start == 1):
                        continue
                    presentPosition = response_array[1]
                    if(x1 > setPoint):
                        response_array[1] = presentPosition-50
                        response_array[0] = 1
                        if(presentPosition <= 950):
                            x1 = 0
                    else:
                        response_array[1] = presentPosition+50
                        response_array[0] = 1
                        if(presentPosition >= 3150):
                            x1 = 640
                # Pack the array into bytes and send it to the client
                arra_data = struct.pack(f'>{ARRAY_SIZE}i', *response_array)
                conn.sendall(array_data)
                response_array[0] = 0
                cv2.imshow("Ball Detection", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()
