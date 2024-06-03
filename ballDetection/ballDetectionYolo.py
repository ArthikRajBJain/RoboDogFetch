from ultralytics import YOLO

model = YOLO('../libraries/yoloDetection/ball.pt')


results = model(source=0, show=True, conf=0.4, save=False, stream=True) # generator of Results objects
