import time
import serial
import math

class NanoBridge:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)

    def read_imu(self):
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            parts = line.split(",")
            if len(parts) != 9:
                return None
            vals = list(map(float, parts))
            return {
                "ax": vals[0],
                "ay": vals[1],
                "az": vals[2],
                "gx": vals[3],
                "gy": vals[4],
                "gz": vals[5],
                "angx": vals[6],
                "angy": vals[7],
                "angz": vals[8],
            }
        except:
            return None

    def send_led(self, state):
        try:
            self.ser.write((state.strip() + "\n").encode("utf-8"))
        except:
            pass


class UnoBridge:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)
        self.last_payload = None

    def read_gas(self):
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("GAS:"):
                return int(line[4:])
        except:
            return None
        return None

    def send_lcd(self, line1, line2, buzzer="OFF"):
        line1 = (line1[:16]).ljust(16)
        line2 = (line2[:16]).ljust(16)
        payload = f"L1:{line1}|L2:{line2}|BZ:{buzzer}\n"

        if payload == self.last_payload:
            return
        self.last_payload = payload

        try:
            self.ser.write(payload.encode("utf-8"))
        except:
            pass


def path_length(points):
    if not points or len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def build_lcd_message(astronaut_id, astronauts, helper_assignments, shortest_paths, gas_value):
    me = astronaut_id
    my_status = astronauts[me]["status"]

    if my_status in ("danger", "danger_unassisted"):
        helper_id = helper_assignments.get(me)
        if helper_id is not None:
            sp = shortest_paths.get(me, [])
            dist = path_length(sp) if sp else 0.0

            if dist <= 1.0:
                return ("DANGER!", f"A{helper_id} {dist:.1f}m away", "NEAR")

            return ("DANGER!", f"A{helper_id} {dist:.1f}m away", "DANGER")

        return ("DANGER!", "NO HELPER", "DANGER")

    target_id = None
    for did, sid in helper_assignments.items():
        if sid == me:
            target_id = did
            break

    if my_status == "helper" and target_id is not None:
        sp = shortest_paths.get(target_id, [])
        dist = path_length(sp) if sp else 0.0
        return (f"GO HELP A{target_id}", f"Dist {dist:.1f}m", "HELP")

    return ("Gas Level:", str(gas_value), "OFF")