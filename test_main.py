import pygame
import math
import heapq
import threading
import queue
import time

from hardware_bridge import NanoBridge, UnoBridge, build_lcd_message
from test_object_detect import TestDetector as Detector

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
manual_override_1 = None   # None / "danger" / "safe"
manual_override_2 = None
manual_override_3 = None

GAS_DANGER_THRESHOLD = 300
GAS_SAFE_THRESHOLD = 250
gas_in_danger = False

gas_value = 0
camera_person_detected = False
# camera_person_count = 0

MOVE_SPEED = 0.04
TURN_SPEED = 2.5
GRAPH_CONNECT_R = 0.40
GAP_BRIDGE_PENALTY = 2.5
LOS_SAMPLE_STEP = 0.20
LOS_CLEAR_RADIUS = 0.45

checkpoints = []
hub_alert = False
transmit_signal = False

PATH_BASE_COLOR = {1:(0,150,40), 2:(30,80,220), 3:(190,110,0)}
CP_COLOR        = {1:(160,255,120), 2:(100,190,255), 3:(255,210,80)}

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

def line_of_sight(p1, p2, nodes):
    x1,y1 = p1
    x2,y2 = p2
    d = math.hypot(x2-x1, y2-y1)
    if d < 1e-6:
        return True
    steps = max(2, int(d / LOS_SAMPLE_STEP) + 1)
    r2 = LOS_CLEAR_RADIUS ** 2
    for k in range(steps + 1):
        t = k / steps
        sx = x1 + t * (x2 - x1)
        sy = y1 + t * (y2 - y1)
        if not any((sx-nx)**2 + (sy-ny)**2 <= r2 for nx,ny in nodes):
            return False
    return True

def theta_star_smooth(path, nodes):
    if len(path) < 3:
        return path
    s = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(path[i], path[j], nodes):
                break
            j -= 1
        s.append(path[j])
        i = j
    return s

class UF:
    def __init__(self, n):
        self.p = list(range(n))
    def find(self, x):
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x
    def union(self, a, b):
        a, b = self.find(a), self.find(b)
        if a != b:
            self.p[a] = b
    def same(self, a, b):
        return self.find(a) == self.find(b)
    def components(self, n):
        g = {}
        for i in range(n):
            g.setdefault(self.find(i), []).append(i)
        return list(g.values())


def build_and_solve():
    global shortest_paths, bridge_segments
    shortest_paths = {}
    bridge_segments = {}
    if not helper_assignments:
        return

    nodes = []
    path_slice = {}
    cur_idx = {}

    for aid, a in astronauts.items():
        s = len(nodes)
        nodes.extend(a["path"])
        path_slice[aid] = (s, len(nodes))

    for wx, wy, _ in checkpoints:
        nodes.append((wx, wy))

    for aid, a in astronauts.items():
        cur_idx[aid] = len(nodes)
        nodes.append((a["x"], a["y"]))

    n = len(nodes)
    adj = [[] for _ in range(n)]
    uf = UF(n)

    def add_edge(i, j, c):
        adj[i].append((j, c))
        adj[j].append((i, c))
        uf.union(i, j)

    gcr2 = GRAPH_CONNECT_R ** 2

    for aid in astronauts:
        s, e = path_slice[aid]
        for i in range(s, e - 1):
            add_edge(i, i + 1, hyp(nodes[i][0], nodes[i][1], nodes[i+1][0], nodes[i+1][1]))
        last = e - 1
        if last >= s:
            ci = cur_idx[aid]
            add_edge(ci, last, hyp(nodes[ci][0], nodes[ci][1], nodes[last][0], nodes[last][1]))

    for i in range(n):
        xi, yi = nodes[i]
        for j in range(i + 1, n):
            dx = xi - nodes[j][0]
            dy = yi - nodes[j][1]
            d2 = dx*dx + dy*dy
            if d2 <= gcr2:
                d = math.sqrt(d2)
                if not uf.same(i, j):
                    add_edge(i, j, d)
                elif (j, d) not in adj[i]:
                    adj[i].append((j, d))
                    adj[j].append((i, d))

    bridge_edges = []
    comps = uf.components(n)
    if len(comps) > 1:
        pairs = []
        for ci in range(len(comps)):
            for cj in range(ci + 1, len(comps)):
                bd = float("inf")
                bi = bj = -1
                for i in comps[ci]:
                    xi, yi = nodes[i]
                    for j in comps[cj]:
                        dx = xi - nodes[j][0]
                        dy = yi - nodes[j][1]
                        d = math.sqrt(dx*dx + dy*dy)
                        if d < bd:
                            bd, bi, bj = d, i, j
                pairs.append((bd, bi, bj))
        pairs.sort()
        for d, i, j in pairs:
            if not uf.same(i, j):
                adj[i].append((j, d * GAP_BRIDGE_PENALTY))
                adj[j].append((i, d * GAP_BRIDGE_PENALTY))
                uf.union(i, j)
                bridge_edges.append((i, j))

    bnp = {(min(i,j), max(i,j)) for i,j in bridge_edges}

    def dijkstra(src, dst):
        INF = float("inf")
        dist = [INF] * n
        dist[src] = 0.0
        prev = [-1] * n
        heap = [(0.0, src)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            if u == dst:
                break
            for v, w in adj[u]:
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        if dist[dst] == INF:
            return None

        path = []
        cur = dst
        while cur != -1:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return [nodes[k] for k in path]

    for did, sid in helper_assignments.items():
        raw = dijkstra(cur_idx[sid], cur_idx[did])
        if raw:
            sm = theta_star_smooth(raw, nodes)
            shortest_paths[did] = sm
            bridge_segments[did] = [
                (sm[i], sm[i+1]) for i in range(len(sm)-1)
                if not line_of_sight(sm[i], sm[i+1], nodes)
            ]
        else:
            fb = [
                (astronauts[sid]["x"], astronauts[sid]["y"]),
                (astronauts[did]["x"], astronauts[did]["y"])
            ]
            shortest_paths[did] = fb
            bridge_segments[did] = [tuple(fb)]    
           
def path_length(pts):
    return sum(hyp(pts[i][0], pts[i][1], pts[i-1][0], pts[i-1][1]) for i in range(1, len(pts)))

def status_color(s):
    return {
        "safe": (0,200,0),
        "danger": (220,50,50),
        "danger_unassisted": (255,140,0),
        "helper": (50,110,255)
    }.get(s, (200,200,200))

def draw_diamond(cx, cy, sz, fill, outline):
    pts = [(cx,cy-sz),(cx+sz,cy),(cx,cy+sz),(cx-sz,cy)]
    pygame.draw.polygon(screen, fill, pts)
    pygame.draw.polygon(screen, outline, pts, 1)

def move_fwd(aid):
    a = astronauts[aid]
    r = math.radians(a["heading"])
    a["x"] += MOVE_SPEED * math.cos(r)
    a["y"] += MOVE_SPEED * math.sin(r)
    record_path(aid)

def move_bwd(aid):
    a = astronauts[aid]
    r = math.radians(a["heading"])
    a["x"] -= MOVE_SPEED * math.cos(r)
    a["y"] -= MOVE_SPEED * math.sin(r)
    record_path(aid)

def turn_l(aid):
    astronauts[aid]["heading"] = (astronauts[aid]["heading"] - TURN_SPEED) % 360

def turn_r(aid):
    astronauts[aid]["heading"] = (astronauts[aid]["heading"] + TURN_SPEED) % 360

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
    global helper_assignments, hub_alert, transmit_signal, path_dirty

    for aid in astronauts:
        astronauts[aid]["status"] = "safe"

    hub_alert = bool(danger_astronaut_ids)
    transmit_signal = len(danger_astronaut_ids) == len(astronauts)

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

    if new_assignments != helper_assignments:
        helper_assignments.clear()
        helper_assignments.update(new_assignments)
        path_dirty = True
    else:
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
            detector.update()
            camera_person_detected = detector.person_detected
            camera_person_count = detector.person_count
            last_vision_time = now
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
    # --- Danger logic for astronaut 1 ---

    if gas_value >= GAS_DANGER_THRESHOLD:
        gas_in_danger = True
    elif gas_value <= GAS_SAFE_THRESHOLD:
        gas_in_danger = False

    gas_danger = gas_in_danger
    obj_danger = camera_person_count >= 1
    # vib_danger = imu_vibration_danger(imu)
    vib_danger = False
    
    auto_danger_1 = gas_danger or obj_danger or vib_danger

    # astronaut 1
    if manual_override_1 == "danger":
        final_danger_1 = True
    elif manual_override_1 == "safe":
        final_danger_1 = False
    else:
        final_danger_1 = auto_danger_1

    # astronaut 2
    if manual_override_2 == "danger":
        final_danger_2 = True
    elif manual_override_2 == "safe":
        final_danger_2 = False
    else:
        final_danger_2 = False

    # astronaut 3
    if manual_override_3 == "danger":
        final_danger_3 = True
    elif manual_override_3 == "safe":
        final_danger_3 = False
    else:
        final_danger_3 = False

    danger_astronaut_ids.clear()
    if final_danger_1:
        danger_astronaut_ids.add(1)
    if final_danger_2:
        danger_astronaut_ids.add(2)
    if final_danger_3:
        danger_astronaut_ids.add(3)
        # danger_astronaut_ids.discard(1)

    # --- Events ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            # astronaut 1
            if event.key == pygame.K_1:
                manual_override_1 = "danger"
            elif event.key == pygame.K_2:
                manual_override_1 = "safe"
            elif event.key == pygame.K_3:
                manual_override_1 = None

            # astronaut 2
            elif event.key == pygame.K_4:
                manual_override_2 = "danger"
            elif event.key == pygame.K_5:
                manual_override_2 = "safe"
            elif event.key == pygame.K_6:
                manual_override_2 = None

            # astronaut 3
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

    # update_assignments()
    # build_simple_paths()
    update_assignments()
    if path_dirty or (helper_assignments and frame_count % PATH_RECOMPUTE_FREQ == 0):
        build_and_solve()
        path_dirty = False

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

        # --- Draw trails ---
    for aid, a in astronauts.items():
        p = a["path"]
        if len(p) < 2:
            continue
        base = PATH_BASE_COLOR[aid]
        total = len(p)
        pts = [w2s(px, py) for px, py in p]
        for i in range(1, total):
            t = i / total
            c = tuple(int(ch * (0.15 + 0.85 * t)) for ch in base)
            pygame.draw.line(screen, c, pts[i-1], pts[i], 1)

    # --- Draw checkpoints ---
    for wx, wy, owner in checkpoints:
        px, py = w2s(wx, wy)
        col = CP_COLOR.get(owner, (255,255,100))
        draw_diamond(px, py, 8, col, (255,255,255))
        screen.blit(sm_font.render(f"CP{owner}", True, (10,10,10)), (px-9, py-7))

    # --- Draw shortest paths ---
    for did, sp in shortest_paths.items():
        if len(sp) < 2:
            continue
        bset = set()
        for p1, p2 in bridge_segments.get(did, []):
            bset.add((p1, p2))
            bset.add((p2, p1))
        spts = [w2s(p[0], p[1]) for p in sp]
        for i in range(1, len(spts)):
            is_b = (sp[i-1], sp[i]) in bset
            pygame.draw.line(screen, (255,160,0) if is_b else (0,210,215), spts[i-1], spts[i], 2)
        for pt in spts[1:-1]:
            pygame.draw.circle(screen, (0,180,200), pt, 3)

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
    screen.blit(font.render(f"Manual override A1: {manual_override_1}", True, (255,255,255)), (20, y)); y += 28
    screen.blit(font.render(f"Gas latched danger: {gas_in_danger}", True, (255,255,255)), (20, y)); y += 28

    if helper_assignments:
        for did, sid in helper_assignments.items():
            sp = shortest_paths.get(did, [])
            info = f" ({path_length(sp):.2f}m)" if sp else ""
            screen.blit(font.render(f"A{sid} -> A{did}{info}", True, (50,200,255)), (20, y))
            y += 28

    if detector.last_frame is not None:
        frame = detector.last_frame
        frame = pygame.image.frombuffer(frame.tobytes(), frame.shape[1::-1], "BGR")
        frame = pygame.transform.scale(frame, (320, 240))
        screen.blit(frame, (1160, 20))

    pygame.display.flip()

pygame.quit()
detector.close()