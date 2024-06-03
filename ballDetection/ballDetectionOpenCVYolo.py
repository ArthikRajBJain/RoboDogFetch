import cv2
from ultralytics import YOLO

model = YOLO('../libraries/yoloDetection/ball.pt')

cap = cv2.VideoCapture(2)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(source=frame, conf=0.65, max_det=1, iou=0.5, imgsz=(256,160))
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        # 640x480
        print(results[0].boxes.xyxy.ravel().cpu().numpy())
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
