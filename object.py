import cv2
from ultralytics import YOLO

# Load a model — use a small model for Pi's limited compute
model = YOLO("yolo11n_ncnn_model")  # 'n' = nano, smallest/fastest variant

# Open the Pi camera (or USB webcam)
# Use 0 for USB cam, or the libcamera pipeline for Pi Camera Module
cap = cv2.VideoCapture(0)

# Optional: lower resolution for better performance
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 60)

cv2.namedWindow("YOLO", cv2.WINDOW_NORMAL);
cv2.setWindowProperty("YOLO", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the frame
    results = model(frame, classes=[0], stream=True, verbose=False)

    # Annotate and display
    for result in results:
        annotated = result.plot()  # draws boxes + labels on frame
        cv2.imshow("YOLO Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
