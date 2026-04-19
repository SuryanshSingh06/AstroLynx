import cv2
from ultralytics import YOLO

class Detector:
    def __init__(self):
        self.model = YOLO("yolo11n_ncnn_model")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.person_detected = False
        self.person_count = 0
        self.last_frame = None

    def update(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.person_detected = False
        self.person_count = 0

        results = self.model(frame, classes=[0], stream=True, verbose=False)

        annotated = frame
        for result in results:
            count = len(result.boxes)
            if count > 0:
                self.person_detected = True
                self.person_count = count
            annotated = result.plot()

        self.last_frame = annotated

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()