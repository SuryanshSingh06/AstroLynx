import math
import time

try:
    import serial
except ImportError:
    serial = None


class LCDBridge:
    def __init__(self, port=None, baud=115200, startup_delay=2.0):
        self.ser = None
        self.last_payload = None

        if port and serial is not None:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                # macOS often resets Arduino boards when serial opens
                time.sleep(startup_delay)
                print(f"LCD connected on {port}")
            except Exception as e:
                print(f"LCD serial unavailable: {e}")

    def send(self, line1: str, line2: str):
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


def path_length(points):
    if not points or len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def build_lcd_message(astronaut_id, astronauts, helper_assignments, shortest_paths):
    me = astronaut_id
    my_status = astronauts[me]["status"]

    if my_status in ("danger", "danger_unassisted"):
        helper_id = helper_assignments.get(me)

        if helper_id is not None:
            sp = shortest_paths.get(me, [])
            dist = path_length(sp) if sp else 0.0
            return (
                "DANGER!",
                f"A{helper_id} {dist:.1f}m away"
            )

        return (
            "DANGER!",
            "NO HELPER"
        )

    target_id = None
    for did, sid in helper_assignments.items():
        if sid == me:
            target_id = did
            break

    if my_status == "helper" and target_id is not None:
        sp = shortest_paths.get(target_id, [])
        dist = path_length(sp) if sp else 0.0
        return (
            f"GO HELP A{target_id}",
            f"Dist {dist:.1f}m"
        )

    hub_dist = math.hypot(astronauts[me]["x"], astronauts[me]["y"])
    return (
        "STATUS: SAFE",
        f"Hub {hub_dist:.1f}m"
    )