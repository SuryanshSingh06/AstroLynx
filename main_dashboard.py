import pygame
import math
import heapq
import threading
import queue
import time

from hardware_bridge import NanoBridge, UnoBridge, build_lcd_message
from object_detect import Detector

pygame.init()

WIDTH, HEIGHT = 1500, 780
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("AstroLynx Final")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
sm_font = pygame.font.SysFont(None, 18)

CENTER_X, CENTER_Y = 575, HEIGHT // 2
SCALE = 60

# NANO_PORT = "/dev/cu.usbserial-XXXX"
# UNO_PORT  = "/dev/cu.usbmodemXXXX"

NANO_PORT = "/dev/ttyUSB0"
UNO_PORT  = "/dev/ttyACM0"

nano = NanoBridge(NANO_PORT)
uno  = UnoBridge(UNO_PORT)
detector = Detector()

IMU_ASTRO_ID = 1
PATH_RECORD_DIST = 0.08
MAX_PATH_POINTS = 400
PATH_RECOMPUTE_FREQ = 15
MAX_TILT_DEG = 30.0
MAX_SPEED = 3.0
MAP_LIMIT = 8.0
VISION_ENABLED = True
VISION_INTERVAL = 0.3   # seconds, so about 3.3 detections/sec
last_vision_time = 0

astronauts = {
    1: {"x": 0.0, "y": 0.0, "heading": 0.0, "status":"safe", "path":[(0.0, 0.0)]},
    2: {"x": 2.0, "y": 1.0, "heading":90.0, "status":"safe", "path":[(2.0, 1.0)]},
    3: {"x":-2.0, "y":-1.0,"heading":45.0, "status":"safe", "path":[(-2.0,-1.0)]},
}

danger_astronaut_ids = set()
helper_assignments = {}
shortest_paths = {}
bridge_segments = {}
frame_count = 0
path_dirty = False

# manual_danger = False
manual_override = None   # None / "danger" / "safe"
manual_override_2 = None
manual_override_3 = None
GAS_DANGER_THRESHOLD = 300
GAS_SAFE_THRESHOLD = 250   # hysteresis so it doesn't flap near 300
gas_in_danger = False

gas_value = 0
camera_person_detected = False
camera_person_count = 0

def hyp(x1,y1,x2,y2):
    return math.hypot(x1-x2, y1-y2)

def w2s(wx,wy):
    return (int(CENTER_X+wx*SCALE), int(CENTER_Y-wy*SCALE))

def dist_between(a1,a2):
    return hyp(a1["x"],a1["y"],a2["x"],a2["y"])

def dist_from_hub(aid):
    return math.hypot(astronauts[aid]["x"], astronauts[aid]["y"])

def record_path(aid):
    a = astronauts[aid]
    p = a["path"]
    if p:
        lx, ly = p[-1]
        if hyp(a["x"], a["y"], lx, ly) >= PATH_RECORD_DIST:
            p.append((a["x"], a["y"]))
            if len(p) > MAX_PATH_POINTS:
                del p[0]

def apply_imu_to_astronaut(aid, imu, dt):
    if imu is None:
        return

    raw_x = imu["angx"]
    raw_y = imu["angy"]

    tgt_vx = (max(-MAX_TILT_DEG, min(MAX_TILT_DEG, raw_y)) / MAX_TILT_DEG) * MAX_SPEED
    tgt_vy = (max(-MAX_TILT_DEG, min(MAX_TILT_DEG, raw_x)) / MAX_TILT_DEG) * MAX_SPEED

    a = astronauts[aid]
    old_x, old_y = a["x"], a["y"]

    new_x = max(-MAP_LIMIT, min(MAP_LIMIT, old_x + tgt_vx * dt))
    new_y = max(-MAP_LIMIT, min(MAP_LIMIT, old_y + tgt_vy * dt))

    a["x"] = new_x
    a["y"] = new_y

    spd = math.hypot(tgt_vx, tgt_vy)
    if spd > 0.001:
        a["heading"] = math.degrees(math.atan2(tgt_vy, tgt_vx)) % 360

    if hyp(new_x, new_y, old_x, old_y) >= PATH_RECORD_DIST:
        record_path(aid)

def imu_vibration_danger(imu):
    if imu is None:
        return False
    return abs(imu["gx"]) > 3.0 or abs(imu["gy"]) > 3.0 or abs(imu["gz"]) > 3.0

def update_assignments():
    global helper_assignments
    for aid in astronauts:
        astronauts[aid]["status"] = "safe"

    for did in danger_astronaut_ids:
        astronauts[did]["status"] = "danger"

    safe_ids = [aid for aid in astronauts if aid not in danger_astronaut_ids]
    new_assignments = {}

    if safe_ids:
        pairs = sorted(
            (dist_between(astronauts[d], astronauts[s]), d, s)
            for d in danger_astronaut_ids for s in safe_ids
        )
        used_d = set()
        used_h = set()
        for _, d, s in pairs:
            if d not in used_d and s not in used_h:
                new_assignments[d] = s
                used_d.add(d)
                used_h.add(s)

    helper_assignments.clear()
    helper_assignments.update(new_assignments)

    for did, sid in helper_assignments.items():
        astronauts[sid]["status"] = "helper"

    for did in danger_astronaut_ids:
        if did not in helper_assignments:
            astronauts[did]["status"] = "danger_unassisted"

def build_simple_paths():
    shortest_paths.clear()
    for did, sid in helper_assignments.items():
        shortest_paths[did] = [
            (astronauts[sid]["x"], astronauts[sid]["y"]),
            (astronauts[did]["x"], astronauts[did]["y"])
        ]

running = True
prev_time = time.time()

while running:
    now = time.time()
    dt = min(now - prev_time, 0.05)
    prev_time = now
    frame_count += 1
    clock.tick(30)

    screen.fill((20,20,30))

    # --- Read hardware ---
    imu = nano.read_imu()
    apply_imu_to_astronaut(1, imu, dt)

    gas_read = uno.read_gas()
    if gas_read is not None:
        gas_value = gas_read

        # --- Vision update (throttled) ---
    if VISION_ENABLED:
        if now - last_vision_time >= VISION_INTERVAL:
            print("VISION BLOCK ENTERED", flush=True)
            detector.update()
            print("DETECTOR UPDATE FINISHED", flush=True)

            camera_person_detected = detector.person_detected
            camera_person_count = detector.person_count
            last_vision_time = now

            print("detected:", camera_person_detected,
                "count:", camera_person_count,
                "manual:", manual_override,
                flush=True)
    else:
        camera_person_detected = False
        camera_person_count = 0
    # camera_person_count = 0


    # --- Danger logic for astronaut 1 ---
    # gas_danger = gas_value > 300
    # obj_danger = camera_person_count >= 1
    # vib_danger = imu_vibration_danger(imu)

    # if gas_danger or obj_danger or vib_danger or manual_danger:
    #     danger_astronaut_ids.add(1)
    # else:
    #     danger_astronaut_ids.discard(1)

    # --- Danger logic for astronaut 1 ---

    # Gas hysteresis:
    # enter danger only when gas rises above GAS_DANGER_THRESHOLD
    # return to safe only when gas falls below GAS_SAFE_THRESHOLD
    if gas_value >= GAS_DANGER_THRESHOLD:
        gas_in_danger = True
    elif gas_value <= GAS_SAFE_THRESHOLD:
        gas_in_danger = False

    gas_danger = gas_in_danger
    obj_danger = camera_person_count >= 1
    vib_danger = imu_vibration_danger(imu)

    auto_danger = gas_danger or obj_danger or vib_danger

    # manual_override:
    # None     -> use sensors normally
    # "danger" -> force danger
    # "safe"   -> force safe even if a sensor says danger
    if manual_override == "danger":
        final_danger = True
    elif manual_override == "safe":
        final_danger = False
    else:
        final_danger = auto_danger

    if final_danger:
        danger_astronaut_ids.add(1)
    else:
        danger_astronaut_ids.discard(1)

    # --- Events ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:

            # Astronaut 1
            if event.key == pygame.K_1:
                manual_override = "danger"

            elif event.key == pygame.K_2:
                manual_override = "safe"

            elif event.key == pygame.K_3:
                manual_override = None


            # Astronaut 2
            elif event.key == pygame.K_4:
                manual_override_2 = "danger"

            elif event.key == pygame.K_5:
                manual_override_2 = "safe"

            elif event.key == pygame.K_6:
                manual_override_2 = None


            # Astronaut 3
            elif event.key == pygame.K_7:
                manual_override_3 = "danger"

            elif event.key == pygame.K_8:
                manual_override_3 = "safe"

            elif event.key == pygame.K_9:
                manual_override_3 = None

            elif event.key == pygame.K_v:
                VISION_ENABLED = not VISION_ENABLED
                if not VISION_ENABLED:
                    camera_person_detected = False
                    camera_person_count = 0

    # controls for 2 and 3
    keys = pygame.key.get_pressed()
    if keys[pygame.K_j]: astronauts[2]["heading"] -= 2.5
    if keys[pygame.K_l]: astronauts[2]["heading"] += 2.5
    if keys[pygame.K_i]:
        r = math.radians(astronauts[2]["heading"])
        astronauts[2]["x"] += 0.04 * math.cos(r)
        astronauts[2]["y"] += 0.04 * math.sin(r)

    if keys[pygame.K_f]: astronauts[3]["heading"] -= 2.5
    if keys[pygame.K_h]: astronauts[3]["heading"] += 2.5
    if keys[pygame.K_t]:
        r = math.radians(astronauts[3]["heading"])
        astronauts[3]["x"] += 0.04 * math.cos(r)
        astronauts[3]["y"] += 0.04 * math.sin(r)

    # Manual danger overrides for astronaut 2
    if manual_override_2 == "danger":
        danger_astronaut_ids.add(2)
    elif manual_override_2 == "safe":
        danger_astronaut_ids.discard(2)

    # Manual danger overrides for astronaut 3
    if manual_override_3 == "danger":
        danger_astronaut_ids.add(3)
    elif manual_override_3 == "safe":
        danger_astronaut_ids.discard(3)

    update_assignments()
    build_simple_paths()

    # --- LED output for astronaut 1 on Nano ---
    if astronauts[1]["status"] in ("danger", "danger_unassisted"):
        nano.send_led("DANGER")
    elif astronauts[1]["status"] == "helper":
        nano.send_led("HELPER")
    else:
        nano.send_led("SAFE")

    # --- LCD + buzzer output for astronaut 1 on Uno ---
    line1, line2, buzzer = build_lcd_message(
        1, astronauts, helper_assignments, shortest_paths, gas_value
    )
    uno.send_lcd(line1, line2, buzzer)

    # --- Draw ---
    for gx in range(0, CENTER_X*2, SCALE):
        pygame.draw.line(screen, (40,40,50), (gx,0), (gx,HEIGHT))
    for gy in range(0, HEIGHT, SCALE):
        pygame.draw.line(screen, (40,40,50), (0,gy), (CENTER_X*2,gy))

    pygame.draw.circle(screen, (255,255,0), (CENTER_X, CENTER_Y), 12)
    screen.blit(font.render("BASE", True, (255,255,0)), (CENTER_X+15, CENTER_Y-10))

    status_colors = {
        "safe": (0,200,0),
        "danger": (220,50,50),
        "danger_unassisted": (255,140,0),
        "helper": (50,110,255)
    }

    for aid, a in astronauts.items():
        px, py = w2s(a["x"], a["y"])
        col = status_colors.get(a["status"], (200,200,200))
        pygame.draw.circle(screen, col, (px, py), 14)

        hx = px + int(24 * math.cos(math.radians(a["heading"])))
        hy = py - int(24 * math.sin(math.radians(a["heading"])))
        pygame.draw.line(screen, (255,255,255), (px,py), (hx,hy), 2)

        label = f"A{aid} {a['status']} d={dist_from_hub(aid):.2f}"
        screen.blit(font.render(label, True, (255,255,255)), (px+15, py))

    y = 20
    screen.blit(font.render(f"Gas: {gas_value}", True, (255,255,255)), (20, y)); y += 28
    screen.blit(font.render(f"Camera person: {camera_person_count}", True, (255,255,255)), (20, y)); y += 28
    # screen.blit(font.render(f"Manual danger: {manual_danger}", True, (255,255,255)), (20, y)); y += 28
    # screen.blit(font.render(f"Manual override: {manual_override}", True, (255,255,255)), (20, y)); y += 28
    # screen.blit(font.render(f"Manual override A1: {manual_override_1}", True, (255,255,255)), (20, y)); y += 28
    screen.blit(font.render(f"Manual override A1: {manual_override}", True, (255,255,255)), (20, y)); y += 28
    screen.blit(font.render(f"Gas latched danger: {gas_in_danger}", True, (255,255,255)), (20, y)); y += 28

    if detector.last_frame is not None:
        frame = detector.last_frame
        frame = pygame.image.frombuffer(frame.tobytes(), frame.shape[1::-1], "BGR")
        frame = pygame.transform.scale(frame, (320, 240))
        screen.blit(frame, (1160, 20))

    pygame.display.flip()

pygame.quit()
detector.close()