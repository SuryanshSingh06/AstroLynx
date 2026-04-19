import pygame
import math
import heapq
import requests
import io
from PIL import Image

pygame.init()

WIDTH, HEIGHT = 1500, 780
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Astronaut Hub Map - Component Bridge Pathfinding + Live Camera")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
sm_font = pygame.font.SysFont(None, 18)

CENTER_X, CENTER_Y = 575, HEIGHT // 2
SCALE = 60

MOVE_SPEED          = 0.04
TURN_SPEED          = 2.5
PATH_RECORD_DIST    = 0.10
MAX_PATH_POINTS     = 400
GRAPH_CONNECT_R     = 0.40
GAP_BRIDGE_PENALTY  = 2.5
LOS_SAMPLE_STEP     = 0.20
LOS_CLEAR_RADIUS    = 0.45
PATH_RECOMPUTE_FREQ = 15

# =========================
# Camera / Pi settings
# =========================
PI_BASE_URL = "http://192.168.0.123:5000"   # <-- CHANGE THIS
CAMERA_TARGET_AID = 1
CAMERA_POLL_EVERY = 15

camera_surface = None
camera_person_detected = False
camera_person_count = 0
camera_poll_counter = 0

# =========================
# State
# =========================
astronauts = {
    1: {"x":  0.0, "y":  0.0, "heading":  0.0, "status": "safe", "path": [(0.0,  0.0)]},
    2: {"x":  2.0, "y":  1.0, "heading": 90.0, "status": "safe", "path": [(2.0,  1.0)]},
    3: {"x": -2.0, "y": -1.0, "heading": 45.0, "status": "safe", "path": [(-2.0,-1.0)]},
}

checkpoints = []
danger_astronaut_ids = set()
helper_assignments   = {}
shortest_paths       = {}
bridge_segments      = {}
hub_alert            = False
transmit_signal      = False
frame_count          = 0
path_dirty           = False

PATH_BASE_COLOR = {1: (0, 150, 40), 2: (30, 80, 220), 3: (190, 110, 0)}
CP_COLOR        = {1: (160, 255, 120), 2: (100, 190, 255), 3: (255, 210, 80)}

def status_color(s):
    return {
        "safe": (0, 200, 0),
        "danger": (220, 50, 50),
        "danger_unassisted": (255, 140, 0),
        "helper": (50, 110, 255)
    }.get(s, (200, 200, 200))

def w2s(wx, wy):
    return (int(CENTER_X + wx * SCALE), int(CENTER_Y - wy * SCALE))

def hyp(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)

def dist_between(a1, a2):
    return hyp(a1["x"], a1["y"], a2["x"], a2["y"])

def dist_from_hub(aid):
    return math.hypot(astronauts[aid]["x"], astronauts[aid]["y"])

def path_length(pts):
    return sum(
        hyp(pts[i][0], pts[i][1], pts[i - 1][0], pts[i - 1][1])
        for i in range(1, len(pts))
    )

# =========================
# Camera helpers
# =========================
def update_camera_feed():
    global camera_surface, camera_person_detected, camera_person_count

    try:
        status = requests.get(f"{PI_BASE_URL}/status", timeout=0.4).json()
        camera_person_detected = status.get("person_detected", False)
        camera_person_count = status.get("person_count", 0)

        img_resp = requests.get(f"{PI_BASE_URL}/frame.jpg", timeout=0.8)
        if img_resp.status_code == 200:
            pil_img = Image.open(io.BytesIO(img_resp.content)).convert("RGB")
            mode = pil_img.mode
            size = pil_img.size
            data = pil_img.tobytes()
            camera_surface = pygame.image.fromstring(data, size, mode)
    except Exception:
        pass

# =========================
# Path recording
# =========================
def record_path(aid):
    a = astronauts[aid]
    path = a["path"]
    if path:
        lx, ly = path[-1]
        if hyp(a["x"], a["y"], lx, ly) >= PATH_RECORD_DIST:
            path.append((a["x"], a["y"]))
            if len(path) > MAX_PATH_POINTS:
                del path[0]
    else:
        path.append((a["x"], a["y"]))

# =========================
# Movement
# =========================
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

# =========================
# Trail-aware LOS
# =========================
def line_of_sight(p1, p2, nodes):
    x1, y1 = p1
    x2, y2 = p2
    seg_len = math.hypot(x2 - x1, y2 - y1)
    if seg_len < 1e-6:
        return True

    steps = max(2, int(seg_len / LOS_SAMPLE_STEP) + 1)
    r2 = LOS_CLEAR_RADIUS ** 2

    for k in range(steps + 1):
        t = k / steps
        sx = x1 + t * (x2 - x1)
        sy = y1 + t * (y2 - y1)
        covered = any((sx - nx) ** 2 + (sy - ny) ** 2 <= r2 for nx, ny in nodes)
        if not covered:
            return False
    return True

def theta_star_smooth(path, nodes):
    if len(path) < 3:
        return path

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(path[i], path[j], nodes):
                break
            j -= 1
        smoothed.append(path[j])
        i = j
    return smoothed

# =========================
# Union-Find
# =========================
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
        groups = {}
        for i in range(n):
            groups.setdefault(self.find(i), []).append(i)
        return list(groups.values())

# =========================
# Graph build + solve
# =========================
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

    for wx, wy, _owner in checkpoints:
        nodes.append((wx, wy))

    for aid, a in astronauts.items():
        cur_idx[aid] = len(nodes)
        nodes.append((a["x"], a["y"]))

    n = len(nodes)
    adj = [[] for _ in range(n)]
    uf = UF(n)

    def add_edge(i, j, cost):
        adj[i].append((j, cost))
        adj[j].append((i, cost))
        uf.union(i, j)

    gcr2 = GRAPH_CONNECT_R ** 2

    for aid in astronauts:
        s, e = path_slice[aid]

        for i in range(s, e - 1):
            d = hyp(nodes[i][0], nodes[i][1], nodes[i + 1][0], nodes[i + 1][1])
            add_edge(i, i + 1, d)

        last = e - 1
        if last >= s:
            ci = cur_idx[aid]
            d = hyp(nodes[ci][0], nodes[ci][1], nodes[last][0], nodes[last][1])
            add_edge(ci, last, d)

    for i in range(n):
        xi, yi = nodes[i]
        for j in range(i + 1, n):
            dx = xi - nodes[j][0]
            dy = yi - nodes[j][1]
            if dx * dx + dy * dy <= gcr2:
                d = math.sqrt(dx * dx + dy * dy)
                if not uf.same(i, j):
                    add_edge(i, j, d)
                elif (j, d) not in adj[i]:
                    adj[i].append((j, d))
                    adj[j].append((i, d))

    bridge_edges_added = []

    comps = uf.components(n)
    if len(comps) > 1:
        inter_pairs = []
        for ci in range(len(comps)):
            for cj in range(ci + 1, len(comps)):
                best_d = float("inf")
                best_i = best_j = -1
                for i in comps[ci]:
                    xi, yi = nodes[i]
                    for j in comps[cj]:
                        dx = xi - nodes[j][0]
                        dy = yi - nodes[j][1]
                        d = math.sqrt(dx * dx + dy * dy)
                        if d < best_d:
                            best_d, best_i, best_j = d, i, j
                inter_pairs.append((best_d, best_i, best_j))

        inter_pairs.sort()

        for d, i, j in inter_pairs:
            if not uf.same(i, j):
                cost = d * GAP_BRIDGE_PENALTY
                adj[i].append((j, cost))
                adj[j].append((i, cost))
                uf.union(i, j)
                bridge_edges_added.append((i, j))

    bridge_node_pairs = set()
    for i, j in bridge_edges_added:
        bridge_node_pairs.add((min(i, j), max(i, j)))

    def dijkstra(src, dst):
        inf = float("inf")
        dist = [inf] * n
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

        if dist[dst] == inf:
            return None, set()

        path = []
        cur = dst
        used_bridges = set()

        while cur != -1:
            path.append(cur)
            p = prev[cur]
            if p >= 0:
                key = (min(cur, p), max(cur, p))
                if key in bridge_node_pairs:
                    used_bridges.add(key)
            cur = p

        path.reverse()
        return [nodes[k] for k in path], used_bridges

    for did, sid in helper_assignments.items():
        raw, used_b = dijkstra(cur_idx[sid], cur_idx[did])

        if raw:
            smoothed = theta_star_smooth(raw, nodes)
            shortest_paths[did] = smoothed
            bsegs = []
            for i in range(len(smoothed) - 1):
                if not line_of_sight(smoothed[i], smoothed[i + 1], nodes):
                    bsegs.append((smoothed[i], smoothed[i + 1]))
            bridge_segments[did] = bsegs
        else:
            shortest_paths[did] = [
                (astronauts[sid]["x"], astronauts[sid]["y"]),
                (astronauts[did]["x"], astronauts[did]["y"]),
            ]
            bridge_segments[did] = [(shortest_paths[did][0], shortest_paths[did][-1])]

# =========================
# Assignment logic
# =========================
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
            [(dist_between(astronauts[did], astronauts[sid]), did, sid)
             for did in danger_astronaut_ids for sid in safe_ids]
        )
        a_d, a_h = set(), set()
        for _, did, sid in pairs:
            if did not in a_d and sid not in a_h:
                new_assignments[did] = sid
                a_d.add(did)
                a_h.add(sid)

    if new_assignments != helper_assignments:
        helper_assignments = new_assignments
        path_dirty = True

    for did, sid in helper_assignments.items():
        astronauts[sid]["status"] = "helper"

    for did in danger_astronaut_ids:
        if did not in helper_assignments:
            astronauts[did]["status"] = "danger_unassisted"

def toggle_danger(aid):
    global path_dirty
    if aid in danger_astronaut_ids:
        danger_astronaut_ids.discard(aid)
    else:
        danger_astronaut_ids.add(aid)
    path_dirty = True

def reset_all():
    global hub_alert, transmit_signal, shortest_paths, bridge_segments, path_dirty
    danger_astronaut_ids.clear()
    helper_assignments.clear()
    shortest_paths = {}
    bridge_segments = {}
    hub_alert = False
    transmit_signal = False
    path_dirty = False

    for aid in astronauts:
        astronauts[aid]["status"] = "safe"
        astronauts[aid]["path"] = [(astronauts[aid]["x"], astronauts[aid]["y"])]

    checkpoints.clear()

def draw_diamond(cx, cy, size, fill, outline):
    pts = [(cx, cy - size), (cx + size, cy), (cx, cy + size), (cx - size, cy)]
    pygame.draw.polygon(screen, fill, pts)
    pygame.draw.polygon(screen, outline, pts, 1)

# =========================
# Main loop
# =========================
running = True
while running:
    frame_count += 1
    clock.tick(60)
    screen.fill((20, 20, 30))

    camera_poll_counter += 1
    if camera_poll_counter >= CAMERA_POLL_EVERY:
        camera_poll_counter = 0
        update_camera_feed()

        if camera_person_detected:
            if CAMERA_TARGET_AID not in danger_astronaut_ids:
                danger_astronaut_ids.add(CAMERA_TARGET_AID)
                path_dirty = True

    for gx in range(0, CENTER_X * 2, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (gx, 0), (gx, HEIGHT))
    for gy in range(0, HEIGHT, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (0, gy), (CENTER_X * 2, gy))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_1:
                toggle_danger(1)
            elif event.key == pygame.K_2:
                toggle_danger(2)
            elif event.key == pygame.K_3:
                toggle_danger(3)
            elif event.key == pygame.K_r:
                reset_all()
            elif event.key == pygame.K_q:
                checkpoints.append((astronauts[1]["x"], astronauts[1]["y"], 1))
                path_dirty = True
            elif event.key == pygame.K_u:
                checkpoints.append((astronauts[2]["x"], astronauts[2]["y"], 2))
                path_dirty = True
            elif event.key == pygame.K_b:
                checkpoints.append((astronauts[3]["x"], astronauts[3]["y"], 3))
                path_dirty = True
            elif event.key == pygame.K_DELETE:
                checkpoints.clear()
                path_dirty = True

    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        turn_l(1)
    if keys[pygame.K_d]:
        turn_r(1)
    if keys[pygame.K_w]:
        move_fwd(1)
    if keys[pygame.K_s]:
        move_bwd(1)

    if keys[pygame.K_j]:
        turn_l(2)
    if keys[pygame.K_l]:
        turn_r(2)
    if keys[pygame.K_i]:
        move_fwd(2)
    if keys[pygame.K_k]:
        move_bwd(2)

    if keys[pygame.K_f]:
        turn_l(3)
    if keys[pygame.K_h]:
        turn_r(3)
    if keys[pygame.K_t]:
        move_fwd(3)
    if keys[pygame.K_g]:
        move_bwd(3)

    update_assignments()
    if path_dirty or (helper_assignments and frame_count % PATH_RECOMPUTE_FREQ == 0):
        build_and_solve()
        path_dirty = False

    # Draw trails
    for aid, a in astronauts.items():
        path = a["path"]
        if len(path) < 2:
            continue

        base = PATH_BASE_COLOR[aid]
        total = len(path)
        pts = [w2s(px, py) for px, py in path]

        for i in range(1, total):
            t = i / total
            c = tuple(int(ch * (0.15 + 0.85 * t)) for ch in base)
            pygame.draw.line(screen, c, pts[i - 1], pts[i], 1)

    # Draw checkpoints
    for wx, wy, owner in checkpoints:
        px, py = w2s(wx, wy)
        col = CP_COLOR.get(owner, (255, 255, 100))
        draw_diamond(px, py, 8, col, (255, 255, 255))
        screen.blit(sm_font.render(f"CP{owner}", True, (10, 10, 10)), (px - 9, py - 7))

    # Draw shortest paths
    for did, sp in shortest_paths.items():
        if len(sp) < 2:
            continue

        bset = set()
        for p1, p2 in bridge_segments.get(did, []):
            bset.add((p1, p2))
            bset.add((p2, p1))

        spts = [w2s(p[0], p[1]) for p in sp]
        for i in range(1, len(spts)):
            key = (sp[i - 1], sp[i])
            is_bridge = key in bset

            if is_bridge:
                pygame.draw.line(screen, (255, 160, 0), spts[i - 1], spts[i], 2)
                seg_px = math.hypot(spts[i][0] - spts[i - 1][0], spts[i][1] - spts[i - 1][1])
                if seg_px > 20:
                    mx = (spts[i - 1][0] + spts[i][0]) // 2
                    my = (spts[i - 1][1] + spts[i][1]) // 2
                    pygame.draw.circle(screen, (255, 200, 80), (mx, my), 3)
            else:
                pygame.draw.line(screen, (0, 210, 215), spts[i - 1], spts[i], 2)

        for pt in spts[1:-1]:
            pygame.draw.circle(screen, (0, 180, 200), pt, 3)

        mid_w = sp[len(sp) // 2]
        mx, my = w2s(mid_w[0], mid_w[1])
        d_lbl = sm_font.render(f"{path_length(sp):.2f}m ({len(sp)} pts)", True, (0, 220, 220))
        bg_r = d_lbl.get_rect(topleft=(mx + 6, my - 10))
        pygame.draw.rect(screen, (10, 10, 20), bg_r.inflate(4, 2))
        screen.blit(d_lbl, (mx + 6, my - 10))

    # Draw hub
    pygame.draw.circle(screen, (255, 255, 0), (CENTER_X, CENTER_Y), 12)
    screen.blit(font.render("BASE (0,0)", True, (255, 255, 0)), (CENTER_X + 15, CENTER_Y - 10))

    # Draw astronauts
    for aid, a in astronauts.items():
        px = int(CENTER_X + a["x"] * SCALE)
        py = int(CENTER_Y - a["y"] * SCALE)
        col = status_color(a["status"])
        hd = dist_from_hub(aid)

        pygame.draw.line(screen, (90, 90, 120), (CENTER_X, CENTER_Y), (px, py), 1)
        pygame.draw.circle(screen, col, (px, py), 14)

        hx = px + int(24 * math.cos(math.radians(a["heading"])))
        hy = py - int(24 * math.sin(math.radians(a["heading"])))
        pygame.draw.line(screen, (255, 255, 255), (px, py), (hx, hy), 2)

        lbl = (
            f"A{aid} | {a['status']} | "
            f"x={a['x']:.2f}, y={a['y']:.2f} | "
            f"heading={a['heading']:.1f}° | dist={hd:.2f}m"
        )
        screen.blit(font.render(lbl, True, (255, 255, 255)), (px + 18, py - 10))

    # HUD
    yt = 20
    if hub_alert:
        screen.blit(font.render("HUB ALERT: Danger detected", True, (255, 80, 80)), (20, yt))
        yt += 28

    if transmit_signal:
        screen.blit(font.render("TRANSMIT STATE: All astronauts in danger", True, (255, 180, 50)), (20, yt))
        yt += 28

    if helper_assignments:
        for did, sid in helper_assignments.items():
            sp = shortest_paths.get(did, [])
            nb = len(bridge_segments.get(did, []))
            info = f"  ({path_length(sp):.2f}m, {nb} bridge{'s' if nb != 1 else ''})" if sp else ""
            screen.blit(font.render(f"A{sid} -> A{did}{info}", True, (50, 200, 255)), (20, yt))
            yt += 28
    elif danger_astronaut_ids and not transmit_signal:
        screen.blit(font.render("No available helper for danger astronauts", True, (255, 180, 50)), (20, yt))
        yt += 28

    if checkpoints:
        screen.blit(font.render(f"Checkpoints: {len(checkpoints)}   [DEL to clear]", True, (220, 220, 80)), (20, yt))
        yt += 28

    legend = [
        "A1: W/S move, A/D turn, Q checkpoint",
        "A2: I/K move, J/L turn, U checkpoint",
        "A3: T/G move, F/H turn, B checkpoint",
        "1/2/3 toggle danger | R reset | DEL clear checkpoints",
        "Live camera appears when astronaut 1 is in danger"
    ]
    for i, line in enumerate(legend):
        screen.blit(font.render(line, True, (180, 180, 180)), (20, HEIGHT - 135 + i * 25))

    # =========================
    # Camera panel
    # =========================
    if CAMERA_TARGET_AID in danger_astronaut_ids:
        panel_x = 1160
        panel_y = 20
        panel_w = 320
        panel_h = 240

        pygame.draw.rect(screen, (30, 30, 40), (panel_x - 10, panel_y - 10, panel_w + 20, panel_h + 70))
        pygame.draw.rect(screen, (120, 120, 140), (panel_x - 10, panel_y - 10, panel_w + 20, panel_h + 70), 2)

        title = font.render("Live Camera Feed", True, (255, 255, 255))
        screen.blit(title, (panel_x, panel_y - 35))

        status_text = f"Person detected: {camera_person_detected} | Count: {camera_person_count}"
        screen.blit(sm_font.render(status_text, True, (255, 200, 120)), (panel_x, panel_y + panel_h + 10))

        if camera_surface is not None:
            scaled = pygame.transform.scale(camera_surface, (panel_w, panel_h))
            screen.blit(scaled, (panel_x, panel_y))
        else:
            pygame.draw.rect(screen, (60, 60, 70), (panel_x, panel_y, panel_w, panel_h))
            screen.blit(sm_font.render("No camera frame", True, (200, 200, 200)), (panel_x + 100, panel_y + 110))

    pygame.display.flip()

pygame.quit()