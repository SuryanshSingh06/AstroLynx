import cv2
import time
from ultralytics import YOLO

class TestDetector:
    def __init__(self):
        self.model = YOLO("yolo11n_ncnn_model")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 10)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.person_detected = False
        self.person_count = 0
        self.last_frame = None

        self.detect_interval = 0.4
        self.last_detect_time = 0

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.last_frame = frame

        now = time.time()
        if now - self.last_detect_time < self.detect_interval:
            return
        self.last_detect_time = now

        results = self.model(frame, classes=[0], verbose=False, imgsz=256)
        result = results[0]

        self.person_count = len(result.boxes)
        self.person_detected = self.person_count > 0

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()