import cv2
import time
from ultralytics import YOLO

try:
    import serial
except ImportError:
    serial = None

# LCDBridge class from message.txt
class LCDBridge:
    def __init__(self, port=None, baud=115200, startup_delay=2.0):
        self.ser = None
        self.last_payload = None

        if port and serial is not None:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                # macOS often resets Arduino boards when serial opens [cite: 2]
                time.sleep(startup_delay)
                print(f"LCD connected on {port}")
            except Exception as e:
                print(f"LCD serial unavailable: {e}")

    def send(self, line1: str, line2: str):
        # Format strictly to 16 characters per line 
        line1 = (line1[:16]).ljust(16)
        line2 = (line2[:16]).ljust(16)
        payload = f"L1:{line1}|L2:{line2}\n"

        if payload == self.last_payload:
            return

        self.last_payload = payload

        if self.ser:
            try:
                self.ser.write(payload.encode("utf-8"))
            except Exception as e:
                print(f"LCD write failed: {e}")

# --- Main Object Detection & Bridge Logic ---

# Initialize the bridge. 
# IMPORTANT: Update 'COM3' or '/dev/ttyACM0' to match your Arduino's port!
bridge = LCDBridge(port='COM3', baud=115200)

# Load a model — use a small model for Pi's limited compute
model = YOLO("yolo11n_ncnn_model") 

# Open the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 60)

cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the frame (class 0 is usually 'person' in COCO)
    results = model(frame, classes=[0], stream=True, verbose=False)
    
    danger_detected = False

    # Annotate and check for detections
    for result in results:
        # If the boxes array has a length greater than 0, an object was found
        if len(result.boxes) > 0:
            danger_detected = True
            
        annotated = result.plot() 
        cv2.imshow("YOLO Detection", annotated)

    # Send the appropriate signal to the Arduino
    if danger_detected:
        bridge.send("DANGER!", "PERSON DETECTED")
    else:
        bridge.send("STATUS: SAFE", "AREA CLEAR")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
