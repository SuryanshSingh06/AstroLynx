import pygame
import math
import heapq

pygame.init()

WIDTH, HEIGHT = 1150, 780
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Astronaut Hub Map - Path Tracking + Shortest Path")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)
small_font = pygame.font.SysFont(None, 18)

CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE = 60  # pixels per meter

MOVE_SPEED = 0.04
TURN_SPEED = 2.5

# Path / graph settings
PATH_SAMPLE_DIST = 0.09     # record waypoint every ~0.09m
MAX_PATH_PTS     = 500      # cap per astronaut
MAX_EDGE_DIST    = 0.38     # max gap to connect two waypoints in graph
REBUILD_INTERVAL = 12       # recompute shortest path every N frames

astronauts = {
    1: {"x": 0.0,  "y": 0.0,  "heading": 0.0,  "status": "safe"},
    2: {"x": 2.0,  "y": 1.0,  "heading": 90.0, "status": "safe"},
    3: {"x": -2.0, "y": -1.0, "heading": 45.0, "status": "safe"},
}

path_history      = {1: [], 2: [], 3: []}   # list of (x, y) per astronaut
checkpoints       = []                       # {"x","y","label","aid"}
danger_astronaut_ids = set()
helper_assignments   = {}                    # danger_id -> helper_id
hub_alert    = False
transmit_signal = False
shortest_paths  = {}                         # danger_id -> list of (x,y)
frame_counter   = 0

# ── Helpers ────────────────────────────────────────────────────────────────

def world_to_screen(x, y):
    return (int(CENTER_X + x * SCALE), int(CENTER_Y - y * SCALE))

def record_path(aid):
    a    = astronauts[aid]
    hist = path_history[aid]
    pos  = (a["x"], a["y"])
    if not hist or math.dist(hist[-1], pos) >= PATH_SAMPLE_DIST:
        hist.append(pos)
        if len(hist) > MAX_PATH_PTS:
            hist.pop(0)

def move_forward(aid, speed=MOVE_SPEED):
    a = astronauts[aid]
    r = math.radians(a["heading"])
    a["x"] += speed * math.cos(r); a["y"] += speed * math.sin(r)
    record_path(aid)

def move_backward(aid, speed=MOVE_SPEED):
    a = astronauts[aid]
    r = math.radians(a["heading"])
    a["x"] -= speed * math.cos(r); a["y"] -= speed * math.sin(r)
    record_path(aid)

def turn_left(aid,  amt=TURN_SPEED): astronauts[aid]["heading"] = (astronauts[aid]["heading"] - amt) % 360
def turn_right(aid, amt=TURN_SPEED): astronauts[aid]["heading"] = (astronauts[aid]["heading"] + amt) % 360

def dist_between(a1, a2):
    return math.sqrt((a1["x"]-a2["x"])**2 + (a1["y"]-a2["y"])**2)

def dist_from_hub(aid):
    a = astronauts[aid]
    return math.sqrt(a["x"]**2 + a["y"]**2)

def set_checkpoint(aid):
    a = astronauts[aid]
    checkpoints.append({"x": a["x"], "y": a["y"],
                        "label": f"CP{len(checkpoints)+1}·A{aid}", "aid": aid})

def toggle_danger(aid):
    if aid in danger_astronaut_ids: danger_astronaut_ids.remove(aid)
    else:                           danger_astronaut_ids.add(aid)
    update_assignments()

def reset_all():
    global hub_alert, transmit_signal
    danger_astronaut_ids.clear(); helper_assignments.clear()
    hub_alert = transmit_signal = False
    shortest_paths.clear()
    for a in astronauts.values(): a["status"] = "safe"

# ── Assignment logic (unchanged) ────────────────────────────────────────────

def update_assignments():
    global helper_assignments, hub_alert, transmit_signal

    for a in astronauts.values(): a["status"] = "safe"
    helper_assignments.clear()
    hub_alert       = len(danger_astronaut_ids) > 0
    transmit_signal = len(danger_astronaut_ids) == len(astronauts)

    for did in danger_astronaut_ids: astronauts[did]["status"] = "danger"

    safe_ids = [aid for aid in astronauts if aid not in danger_astronaut_ids]
    if not safe_ids: return

    pairs = sorted(
        [(dist_between(astronauts[d], astronauts[s]), d, s)
         for d in danger_astronaut_ids for s in safe_ids]
    )
    assigned_d, assigned_s = set(), set()
    for _, did, sid in pairs:
        if did in assigned_d or sid in assigned_s: continue
        helper_assignments[did] = sid
        assigned_d.add(did); assigned_s.add(sid)

    for did, sid in helper_assignments.items(): astronauts[sid]["status"] = "helper"
    for did in danger_astronaut_ids:
        if did not in helper_assignments: astronauts[did]["status"] = "danger_unassisted"

# ── Graph + Dijkstra ─────────────────────────────────────────────────────────

def build_shortest_paths():
    """Build a free-space waypoint graph from trail history + checkpoints,
       then run Dijkstra for each helper→danger pair."""
    global shortest_paths
    shortest_paths = {}
    if not helper_assignments: return

    # ── Collect nodes ──
    nodes = []

    # Sampled path history (every 3rd point to keep graph lean)
    path_node_ranges = {}
    for aid in astronauts:
        hist  = path_history[aid]
        start = len(nodes)
        sampled = [hist[i] for i in range(0, len(hist), 3)]
        if hist and hist[-1] not in sampled: sampled.append(hist[-1])
        nodes.extend(sampled)
        path_node_ranges[aid] = (start, len(nodes), sampled)

    # Checkpoints
    for cp in checkpoints:
        nodes.append((cp["x"], cp["y"]))

    # Current astronaut positions (always included as terminal nodes)
    astro_idx = {}
    for aid, a in astronauts.items():
        astro_idx[aid] = len(nodes)
        nodes.append((a["x"], a["y"]))

    n = len(nodes)
    adj = [[] for _ in range(n)]

    # ── Connect nodes within MAX_EDGE_DIST (spatial proximity) ──
    for i in range(n):
        for j in range(i + 1, n):
            d = math.dist(nodes[i], nodes[j])
            if 0 < d <= MAX_EDGE_DIST:
                adj[i].append((j, d))
                adj[j].append((i, d))

    # ── Guarantee path continuity along each astronaut's trail ──
    for aid, (start, end, sampled) in path_node_ranges.items():
        for k in range(len(sampled) - 1):
            i, j = start + k, start + k + 1
            d = math.dist(nodes[i], nodes[j])
            if d > 0 and not any(nb == j for nb, _ in adj[i]):
                adj[i].append((j, d)); adj[j].append((i, d))

    # ── Dijkstra ──
    def dijkstra(src, dst):
        dist_arr = [math.inf] * n
        prev     = [-1]       * n
        dist_arr[src] = 0.0
        heap = [(0.0, src)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist_arr[u]: continue
            if u == dst:        break
            for v, w in adj[u]:
                nd = d + w
                if nd < dist_arr[v]:
                    dist_arr[v] = nd; prev[v] = u
                    heapq.heappush(heap, (nd, v))
        if math.isinf(dist_arr[dst]): return None
        path, cur = [], dst
        while cur != -1: path.append(nodes[cur]); cur = prev[cur]
        path.reverse(); return path

    for did, sid in helper_assignments.items():
        path = dijkstra(astro_idx[sid], astro_idx[did])
        shortest_paths[did] = path if (path and len(path) >= 2) else [
            (astronauts[sid]["x"], astronauts[sid]["y"]),
            (astronauts[did]["x"], astronauts[did]["y"])
        ]

# ── Color helpers ────────────────────────────────────────────────────────────

def color_for_status(s):
    return {"safe": (0, 200, 0), "danger": (220, 50, 50),
            "danger_unassisted": (255, 140, 0), "helper": (50, 100, 255)}.get(s, (200,200,200))

TRAIL_COLORS = {1: (0, 155, 110), 2: (40, 80, 210), 3: (160, 55, 200)}
CP_COLORS    = {1: (0, 220, 180), 2: (220, 200, 0), 3: (200, 60, 230)}

trail_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

# ── Main loop ────────────────────────────────────────────────────────────────

running = True
while running:
    clock.tick(60)
    frame_counter += 1
    screen.fill((20, 20, 30))

    # grid
    for x in range(0, WIDTH, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (0, y), (WIDTH, y))

    # ── Events ──────────────────────────────────────────────────────────────
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            if   event.key == pygame.K_1: toggle_danger(1)
            elif event.key == pygame.K_2: toggle_danger(2)
            elif event.key == pygame.K_3: toggle_danger(3)
            elif event.key == pygame.K_r: reset_all()
            elif event.key == pygame.K_q: set_checkpoint(1)   # Q → A1 checkpoint
            elif event.key == pygame.K_u: set_checkpoint(2)   # U → A2 checkpoint
            elif event.key == pygame.K_y: set_checkpoint(3)   # Y → A3 checkpoint
            elif event.key == pygame.K_c: checkpoints.clear() # C → clear all checkpoints

    keys = pygame.key.get_pressed()

    # A1: WASD
    if keys[pygame.K_a]: turn_left(1)
    if keys[pygame.K_d]: turn_right(1)
    if keys[pygame.K_w]: move_forward(1)
    if keys[pygame.K_s]: move_backward(1)
    # A2: IJKL
    if keys[pygame.K_j]: turn_left(2)
    if keys[pygame.K_l]: turn_right(2)
    if keys[pygame.K_i]: move_forward(2)
    if keys[pygame.K_k]: move_backward(2)
    # A3: TFGH
    if keys[pygame.K_f]: turn_left(3)
    if keys[pygame.K_h]: turn_right(3)
    if keys[pygame.K_t]: move_forward(3)
    if keys[pygame.K_g]: move_backward(3)

    # ── Logic ───────────────────────────────────────────────────────────────
    update_assignments()
    if frame_counter % REBUILD_INTERVAL == 0:
        build_shortest_paths()

    # ── Draw trails (alpha-faded, older = dimmer) ────────────────────────────
    trail_surf.fill((0, 0, 0, 0))
    for aid in astronauts:
        hist = path_history[aid]
        r, g, b = TRAIL_COLORS[aid]
        n_pts = len(hist)
        if n_pts < 2: continue
        spts = [world_to_screen(x, y) for x, y in hist]
        for i in range(n_pts - 1):
            frac  = i / max(n_pts - 2, 1)
            alpha = int(45 + 160 * frac)          # older segments dimmer
            pygame.draw.line(trail_surf, (r, g, b, alpha), spts[i], spts[i+1], 2)
        # Footstep dots every 6 points
        for i in range(0, n_pts, 6):
            frac  = i / max(n_pts - 1, 1)
            alpha = int(60 + 150 * frac)
            pygame.draw.circle(trail_surf, (r, g, b, alpha), spts[i], 2)
    screen.blit(trail_surf, (0, 0))

    # ── Draw shortest paths (gold, replaces blue helper line) ───────────────
    for did, path in shortest_paths.items():
        if len(path) < 2: continue
        pts = [world_to_screen(x, y) for x, y in path]
        # Glow pass (thicker, dimmer)
        for i in range(len(pts) - 1):
            pygame.draw.line(screen, (120, 90, 0), pts[i], pts[i+1], 5)
        # Core line
        for i in range(len(pts) - 1):
            pygame.draw.line(screen, (255, 215, 0), pts[i], pts[i+1], 2)
        # Waypoint markers along path (skip first/last which are astronauts)
        for pt in pts[1:-1]:
            pygame.draw.circle(screen, (255, 215, 0), pt, 3)

    # ── Draw checkpoints ────────────────────────────────────────────────────
    for cp in checkpoints:
        px, py = world_to_screen(cp["x"], cp["y"])
        col    = CP_COLORS.get(cp["aid"], (200, 200, 200))
        pygame.draw.rect(screen, col, (px - 7, py - 7, 14, 14), 2)
        pygame.draw.line(screen, col, (px - 5, py),     (px + 5, py),     1)
        pygame.draw.line(screen, col, (px,     py - 5), (px,     py + 5), 1)
        screen.blit(small_font.render(cp["label"], True, col), (px + 9, py - 8))

    # ── Hub ──────────────────────────────────────────────────────────────────
    pygame.draw.circle(screen, (255, 255, 0), (CENTER_X, CENTER_Y), 12)
    screen.blit(font.render("BASE (0,0)", True, (255, 255, 0)), (CENTER_X + 15, CENTER_Y - 10))

    # ── Astronauts ───────────────────────────────────────────────────────────
    for aid, a in astronauts.items():
        px, py = world_to_screen(a["x"], a["y"])
        color  = color_for_status(a["status"])
        hub_d  = dist_from_hub(aid)

        pygame.draw.line(screen, (90, 90, 120), (CENTER_X, CENTER_Y), (px, py), 1)
        pygame.draw.circle(screen, color, (px, py), 14)

        hx = px + int(24 * math.cos(math.radians(a["heading"])))
        hy = py - int(24 * math.sin(math.radians(a["heading"])))
        pygame.draw.line(screen, (255, 255, 255), (px, py), (hx, hy), 2)

        label = (f"A{aid} | {a['status']} | "
                 f"x={a['x']:.2f}, y={a['y']:.2f} | "
                 f"heading={a['heading']:.1f}° | dist to hub={hub_d:.2f}m")
        screen.blit(font.render(label, True, (255, 255, 255)), (px + 18, py - 10))

    # ── Status panel ─────────────────────────────────────────────────────────
    y_text = 20
    if hub_alert:
        screen.blit(font.render("HUB ALERT: Danger detected", True, (255, 80, 80)), (20, y_text))
        y_text += 28
    if transmit_signal:
        screen.blit(font.render("TRANSMIT STATE: All astronauts in danger", True, (255, 180, 50)), (20, y_text))
        y_text += 28

    if helper_assignments:
        for did, sid in helper_assignments.items():
            path_info = ""
            if did in shortest_paths:
                p     = shortest_paths[did]
                total = sum(math.dist(p[i], p[i+1]) for i in range(len(p)-1))
                path_info = f"  [shortest path {total:.2f}m via {len(p)} nodes]"
            msg = f"A{sid} → A{did}{path_info}"
            screen.blit(font.render(msg, True, (50, 200, 255)), (20, y_text))
            y_text += 28
    elif danger_astronaut_ids and not transmit_signal:
        screen.blit(font.render("No available helper for danger astronaut(s)", True, (255, 180, 50)), (20, y_text))
        y_text += 28

    # ── Instructions ─────────────────────────────────────────────────────────
    instructions = [
        "A1: W/S move, A/D turn    Q = drop A1 checkpoint",
        "A2: I/K move, J/L turn    U = drop A2 checkpoint",
        "A3: T/G move, F/H turn    Y = drop A3 checkpoint",
        "1/2/3 = toggle danger     R = reset     C = clear checkpoints",
        "Gold line = Dijkstra shortest path through explored trails + checkpoints",
    ]
    for i, line in enumerate(instructions):
        screen.blit(font.render(line, True, (200, 200, 200)), (20, HEIGHT - 140 + i * 25))

    pygame.display.flip()

pygame.quit()