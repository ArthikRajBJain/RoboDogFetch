from ultralytics import YOLO
model = YOLO('ball.pt')


results = model(source=0, show=True, conf=0.4, save=True) # generator of Results objects
