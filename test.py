from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO("yolo11n.pt")
print("Model loaded OK")

# Test with a dummy frame instead of the camera
dummy = np.zeros((480, 640, 3), dtype=np.uint8)
print("Running inference...")
results = model(dummy, verbose=True)
print("Inference OK")
