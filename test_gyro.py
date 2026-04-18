import pygame
import math
import heapq
import threading
import serial
import time
import statistics
import requests
import io
from collections import deque
from PIL import Image

pygame.init()

WIDTH, HEIGHT = 1500, 780
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Astronaut Hub Map - IMU Velocity Control")
clock = pygame.time.Clock()
font    = pygame.font.SysFont(None, 24)
sm_font = pygame.font.SysFont(None, 18)

CENTER_X, CENTER_Y = 575, HEIGHT // 2
SCALE = 60

MOVE_SPEED          = 0.04
TURN_SPEED          = 2.5
PATH_RECORD_DIST    = 0.08
MAX_PATH_POINTS     = 400
GRAPH_CONNECT_R     = 0.40
GAP_BRIDGE_PENALTY  = 2.5
LOS_SAMPLE_STEP     = 0.20
LOS_CLEAR_RADIUS    = 0.45
PATH_RECOMPUTE_FREQ = 15

# ── IMU / Serial ──────────────────────────────────────────────────────────────
SERIAL_PORT  = "/dev/ttyUSB0"
SERIAL_BAUD  = 115200
IMU_ASTRO_ID = 1

# ── IMU smoothing (median + EMA) ──────────────────────────────────────────────
MEDIAN_N  = 9      # rolling window for spike rejection (must be odd)
EMA_ALPHA = 0.20   # how quickly the filtered angle tracks the median (0=frozen,1=raw)
DEADBAND  = 3.0    # degrees — below this treated as zero (sensor noise floor)

# ── Velocity-based movement ───────────────────────────────────────────────────
#
# Tilt angle → speed, NOT position.
# Think of the board as a joystick:
#   • flat             → stop
#   • small tilt       → slow creep
#   • large tilt       → fast movement
#
# MAX_TILT_DEG  : tilt angle that maps to MAX_SPEED (metres/sec)
# MAX_SPEED     : max metres per second at full tilt
# FRICTION      : velocity multiplied by this each frame (0.85 = snappy stop,
#                 0.95 = slides a bit, 0.99 = very slippery)
# MAP_LIMIT     : world boundary in metres (astronaut can't leave this)
#
MAX_TILT_DEG = 30.0    # degrees → full speed
MAX_SPEED    = 3.0     # metres / second at full tilt
FRICTION     = 0.88    # per-frame velocity decay when tilt is in deadband
MAP_LIMIT    = 8.0     # metres from origin

# ── Camera / Pi ───────────────────────────────────────────────────────────────
PI_BASE_URL       = "http://127.0.0.1:5000"
CAMERA_TARGET_AID = 1
CAMERA_POLL_EVERY = 15

camera_surface         = None
camera_person_detected = False
camera_person_count    = 0
camera_poll_counter    = 0

# ── IMU shared state ──────────────────────────────────────────────────────────
imu_lock = threading.Lock()
imu_data = {
    "angX": 0.0, "angY": 0.0, "angZ": 0.0,
    "connected": False, "raw": "",
}

# ── Smoothing state ───────────────────────────────────────────────────────────
_buf   = {k: deque([0.0]*MEDIAN_N, maxlen=MEDIAN_N) for k in ("X","Y")}
f_angX = 0.0   # filtered pitch (→ world Y velocity)
f_angY = 0.0   # filtered roll  (→ world X velocity)

# ── Velocity state for IMU astronaut ─────────────────────────────────────────
vel_x = 0.0
vel_y = 0.0

# ── Astronaut state ───────────────────────────────────────────────────────────
astronauts = {
    1: {"x": 0.0, "y": 0.0, "heading": 0.0, "status": "safe", "path": [(0.0, 0.0)]},
    2: {"x": 2.0, "y": 1.0, "heading":90.0, "status": "safe", "path": [(2.0, 1.0)]},
    3: {"x":-2.0, "y":-1.0, "heading":45.0, "status": "safe", "path": [(-2.0,-1.0)]},
}

checkpoints          = []
danger_astronaut_ids = set()
helper_assignments   = {}
shortest_paths       = {}
bridge_segments      = {}
hub_alert            = False
transmit_signal      = False
frame_count          = 0
path_dirty           = False

PATH_BASE_COLOR = {1:(0,150,40), 2:(30,80,220), 3:(190,110,0)}
CP_COLOR        = {1:(160,255,120), 2:(100,190,255), 3:(255,210,80)}

def status_color(s):
    return {"safe":(0,200,0),"danger":(220,50,50),
            "danger_unassisted":(255,140,0),"helper":(50,110,255)}.get(s,(200,200,200))

def w2s(wx,wy): return (int(CENTER_X+wx*SCALE), int(CENTER_Y-wy*SCALE))
def hyp(x1,y1,x2,y2): return math.hypot(x1-x2,y1-y2)
def dist_between(a1,a2): return hyp(a1["x"],a1["y"],a2["x"],a2["y"])
def dist_from_hub(aid): return math.hypot(astronauts[aid]["x"],astronauts[aid]["y"])
def path_length(pts): return sum(hyp(pts[i][0],pts[i][1],pts[i-1][0],pts[i-1][1]) for i in range(1,len(pts)))

# ── Serial reader ─────────────────────────────────────────────────────────────
def serial_reader():
    while True:
        try:
            with serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1) as ser:
                time.sleep(2)
                with imu_lock:
                    imu_data["connected"] = True
                while True:
                    raw = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not raw or "," not in raw:
                        continue
                    parts = raw.split(",")
                    if len(parts) != 9:
                        continue
                    try:
                        vals = list(map(float, parts))
                    except ValueError:
                        continue
                    # vals: ax,ay,az,gx,gy,gz,angX,angY,angZ
                    with imu_lock:
                        imu_data["angX"] = vals[6]
                        imu_data["angY"] = vals[7]
                        imu_data["angZ"] = vals[8]
                        imu_data["raw"]  = raw
        except (serial.SerialException, OSError):
            with imu_lock:
                imu_data["connected"] = False
            time.sleep(3)

threading.Thread(target=serial_reader, daemon=True).start()

# ── IMU update (velocity-based) ───────────────────────────────────────────────
def apply_imu_to_astronaut(aid, dt):
    """
    Each frame:
      1. Read latest raw angles from serial thread.
      2. Median filter  → kills single-sample spikes.
      3. EMA            → smooths continuous trend.
      4. Deadband       → zero out sensor noise near flat.
      5. Convert angle  → target velocity (proportional, capped).
      6. Apply friction when inside deadband (natural deceleration).
      7. Integrate velocity → new position.
      8. Derive heading from velocity direction (no drifty gyro yaw).
    """
    global f_angX, f_angY, vel_x, vel_y

    with imu_lock:
        raw_X = imu_data["angX"]
        raw_Y = imu_data["angY"]
        conn  = imu_data["connected"]
    if not conn:
        return

    # ── Layer 1: median (spike rejection) ─────────────────────────────────
    _buf["X"].append(raw_X)
    _buf["Y"].append(raw_Y)
    med_X = statistics.median(_buf["X"])
    med_Y = statistics.median(_buf["Y"])

    # ── Layer 2: EMA (trend smoothing) ────────────────────────────────────
    f_angX = (1.0 - EMA_ALPHA)*f_angX + EMA_ALPHA*med_X
    f_angY = (1.0 - EMA_ALPHA)*f_angY + EMA_ALPHA*med_Y

    # ── Layer 3: deadband ─────────────────────────────────────────────────
    eff_X = 0.0 if abs(f_angX) < DEADBAND else f_angX
    eff_Y = 0.0 if abs(f_angY) < DEADBAND else f_angY

    # ── Layer 4: angle → velocity ─────────────────────────────────────────
    # Clamp to ±MAX_TILT_DEG, normalise to -1…+1, scale to MAX_SPEED m/s
    # angY (roll)  → X axis   (tilt right  = move right)
    # angX (pitch) → Y axis   (tilt forward = move up on map)
    tgt_vx =  (max(-MAX_TILT_DEG, min(MAX_TILT_DEG, eff_Y)) / MAX_TILT_DEG) * MAX_SPEED
    tgt_vy =  (max(-MAX_TILT_DEG, min(MAX_TILT_DEG, eff_X)) / MAX_TILT_DEG) * MAX_SPEED

    if eff_X == 0.0 and eff_Y == 0.0:
        # Inside deadband → apply friction, decelerate naturally
        vel_x *= FRICTION
        vel_y *= FRICTION
        # Stop completely below tiny threshold to avoid forever-creep
        if abs(vel_x) < 0.002: vel_x = 0.0
        if abs(vel_y) < 0.002: vel_y = 0.0
    else:
        # Outside deadband → blend current velocity toward target velocity
        # Using same EMA idea: fast response (0.25) feels snappy
        VEL_ALPHA = 0.25
        vel_x = (1.0-VEL_ALPHA)*vel_x + VEL_ALPHA*tgt_vx
        vel_y = (1.0-VEL_ALPHA)*vel_y + VEL_ALPHA*tgt_vy

    # ── Layer 5: integrate position ───────────────────────────────────────
    a = astronauts[aid]
    old_x, old_y = a["x"], a["y"]

    new_x = old_x + vel_x * dt
    new_y = old_y + vel_y * dt

    # Clamp to map boundary
    new_x = max(-MAP_LIMIT, min(MAP_LIMIT, new_x))
    new_y = max(-MAP_LIMIT, min(MAP_LIMIT, new_y))

    a["x"] = new_x
    a["y"] = new_y

    # ── Layer 6: heading from velocity direction ───────────────────────────
    # Derive from movement direction — avoids drifty gyro yaw entirely
    speed = math.hypot(vel_x, vel_y)
    if speed > 0.05:
        a["heading"] = math.degrees(math.atan2(vel_y, vel_x)) % 360

    # ── Record path ───────────────────────────────────────────────────────
    if hyp(new_x, new_y, old_x, old_y) >= PATH_RECORD_DIST:
        a["path"].append((new_x, new_y))
        if len(a["path"]) > MAX_PATH_POINTS:
            del a["path"][0]

# ── Camera ────────────────────────────────────────────────────────────────────
def update_camera_feed():
    global camera_surface, camera_person_detected, camera_person_count
    try:
        s = requests.get(f"{PI_BASE_URL}/status", timeout=0.4).json()
        camera_person_detected = s.get("person_detected", False)
        camera_person_count    = s.get("person_count", 0)
        r = requests.get(f"{PI_BASE_URL}/frame.jpg", timeout=0.8)
        if r.status_code == 200:
            img = Image.open(io.BytesIO(r.content)).convert("RGB")
            camera_surface = pygame.image.fromstring(img.tobytes(), img.size, img.mode)
    except Exception:
        pass

# ── Keyboard movement (A2/A3) ─────────────────────────────────────────────────
def record_path(aid):
    a,p = astronauts[aid], astronauts[aid]["path"]
    if p:
        lx,ly=p[-1]
        if hyp(a["x"],a["y"],lx,ly) >= PATH_RECORD_DIST:
            p.append((a["x"],a["y"]))
            if len(p)>MAX_PATH_POINTS: del p[0]
    else: p.append((a["x"],a["y"]))

def move_fwd(aid):
    a=astronauts[aid]; r=math.radians(a["heading"])
    a["x"]+=MOVE_SPEED*math.cos(r); a["y"]+=MOVE_SPEED*math.sin(r); record_path(aid)
def move_bwd(aid):
    a=astronauts[aid]; r=math.radians(a["heading"])
    a["x"]-=MOVE_SPEED*math.cos(r); a["y"]-=MOVE_SPEED*math.sin(r); record_path(aid)
def turn_l(aid): astronauts[aid]["heading"]=(astronauts[aid]["heading"]-TURN_SPEED)%360
def turn_r(aid): astronauts[aid]["heading"]=(astronauts[aid]["heading"]+TURN_SPEED)%360

# ── LOS + Theta* ──────────────────────────────────────────────────────────────
def line_of_sight(p1,p2,nodes):
    x1,y1=p1; x2,y2=p2
    d=math.hypot(x2-x1,y2-y1)
    if d<1e-6: return True
    steps=max(2,int(d/LOS_SAMPLE_STEP)+1); r2=LOS_CLEAR_RADIUS**2
    for k in range(steps+1):
        t=k/steps; sx=x1+t*(x2-x1); sy=y1+t*(y2-y1)
        if not any((sx-nx)**2+(sy-ny)**2<=r2 for nx,ny in nodes): return False
    return True

def theta_star_smooth(path,nodes):
    if len(path)<3: return path
    s=[path[0]]; i=0
    while i<len(path)-1:
        j=len(path)-1
        while j>i+1:
            if line_of_sight(path[i],path[j],nodes): break
            j-=1
        s.append(path[j]); i=j
    return s

# ── Union-Find ────────────────────────────────────────────────────────────────
class UF:
    def __init__(self,n): self.p=list(range(n))
    def find(self,x):
        while self.p[x]!=x: self.p[x]=self.p[self.p[x]]; x=self.p[x]
        return x
    def union(self,a,b):
        a,b=self.find(a),self.find(b)
        if a!=b: self.p[a]=b
    def same(self,a,b): return self.find(a)==self.find(b)
    def components(self,n):
        g={}
        for i in range(n): g.setdefault(self.find(i),[]).append(i)
        return list(g.values())

# ── Graph + Dijkstra + Theta* ─────────────────────────────────────────────────
def build_and_solve():
    global shortest_paths,bridge_segments
    shortest_paths={}; bridge_segments={}
    if not helper_assignments: return

    nodes=[]; path_slice={}; cur_idx={}
    for aid,a in astronauts.items():
        s=len(nodes); nodes.extend(a["path"]); path_slice[aid]=(s,len(nodes))
    for wx,wy,_ in checkpoints: nodes.append((wx,wy))
    for aid,a in astronauts.items(): cur_idx[aid]=len(nodes); nodes.append((a["x"],a["y"]))

    n=len(nodes); adj=[[] for _ in range(n)]; uf=UF(n)
    def add_edge(i,j,c): adj[i].append((j,c)); adj[j].append((i,c)); uf.union(i,j)

    gcr2=GRAPH_CONNECT_R**2
    for aid in astronauts:
        s,e=path_slice[aid]
        for i in range(s,e-1):
            add_edge(i,i+1,hyp(nodes[i][0],nodes[i][1],nodes[i+1][0],nodes[i+1][1]))
        last=e-1
        if last>=s:
            ci=cur_idx[aid]; add_edge(ci,last,hyp(nodes[ci][0],nodes[ci][1],nodes[last][0],nodes[last][1]))
    for i in range(n):
        xi,yi=nodes[i]
        for j in range(i+1,n):
            dx=xi-nodes[j][0]; dy=yi-nodes[j][1]; d2=dx*dx+dy*dy
            if d2<=gcr2:
                d=math.sqrt(d2)
                if not uf.same(i,j): add_edge(i,j,d)
                elif (j,d) not in adj[i]: adj[i].append((j,d)); adj[j].append((i,d))

    bridge_edges=[]
    comps=uf.components(n)
    if len(comps)>1:
        pairs=[]
        for ci in range(len(comps)):
            for cj in range(ci+1,len(comps)):
                bd=float("inf"); bi=bj=-1
                for i in comps[ci]:
                    xi,yi=nodes[i]
                    for j in comps[cj]:
                        dx=xi-nodes[j][0]; dy=yi-nodes[j][1]; d=math.sqrt(dx*dx+dy*dy)
                        if d<bd: bd,bi,bj=d,i,j
                pairs.append((bd,bi,bj))
        pairs.sort()
        for d,i,j in pairs:
            if not uf.same(i,j):
                adj[i].append((j,d*GAP_BRIDGE_PENALTY)); adj[j].append((i,d*GAP_BRIDGE_PENALTY))
                uf.union(i,j); bridge_edges.append((i,j))

    bnp={(min(i,j),max(i,j)) for i,j in bridge_edges}

    def dijkstra(src,dst):
        INF=float("inf"); dist=[INF]*n; dist[src]=0.0; prev=[-1]*n; heap=[(0.0,src)]
        while heap:
            d,u=heapq.heappop(heap)
            if d>dist[u]: continue
            if u==dst: break
            for v,w in adj[u]:
                nd=d+w
                if nd<dist[v]: dist[v]=nd; prev[v]=u; heapq.heappush(heap,(nd,v))
        if dist[dst]==INF: return None
        path=[]; cur=dst
        while cur!=-1: path.append(cur); cur=prev[cur]
        path.reverse(); return [nodes[k] for k in path]

    for did,sid in helper_assignments.items():
        raw=dijkstra(cur_idx[sid],cur_idx[did])
        if raw:
            sm=theta_star_smooth(raw,nodes); shortest_paths[did]=sm
            bridge_segments[did]=[(sm[i],sm[i+1]) for i in range(len(sm)-1)
                                   if not line_of_sight(sm[i],sm[i+1],nodes)]
        else:
            fb=[(astronauts[sid]["x"],astronauts[sid]["y"]),(astronauts[did]["x"],astronauts[did]["y"])]
            shortest_paths[did]=fb; bridge_segments[did]=[tuple(fb)]

# ── Assignments ───────────────────────────────────────────────────────────────
def update_assignments():
    global helper_assignments,hub_alert,transmit_signal,path_dirty
    for aid in astronauts: astronauts[aid]["status"]="safe"
    hub_alert=bool(danger_astronaut_ids)
    transmit_signal=len(danger_astronaut_ids)==len(astronauts)
    for did in danger_astronaut_ids: astronauts[did]["status"]="danger"
    safe_ids=[aid for aid in astronauts if aid not in danger_astronaut_ids]
    new_a={}
    if safe_ids:
        pairs=sorted([(dist_between(astronauts[d],astronauts[s]),d,s)
                      for d in danger_astronaut_ids for s in safe_ids])
        ad,ah=set(),set()
        for _,d,s in pairs:
            if d not in ad and s not in ah: new_a[d]=s; ad.add(d); ah.add(s)
    if new_a!=helper_assignments: helper_assignments=new_a; path_dirty=True
    for did,sid in helper_assignments.items(): astronauts[sid]["status"]="helper"
    for did in danger_astronaut_ids:
        if did not in helper_assignments: astronauts[did]["status"]="danger_unassisted"

def toggle_danger(aid):
    global path_dirty
    danger_astronaut_ids.discard(aid) if aid in danger_astronaut_ids else danger_astronaut_ids.add(aid)
    path_dirty=True

def reset_all():
    global hub_alert,transmit_signal,shortest_paths,bridge_segments,path_dirty,vel_x,vel_y
    danger_astronaut_ids.clear(); helper_assignments.clear()
    shortest_paths={}; bridge_segments={}; hub_alert=transmit_signal=False; path_dirty=False
    vel_x=vel_y=0.0
    for aid in astronauts:
        astronauts[aid]["status"]="safe"
        astronauts[aid]["path"]=[(astronauts[aid]["x"],astronauts[aid]["y"])]
    checkpoints.clear()

def draw_diamond(cx,cy,sz,fill,outline):
    pts=[(cx,cy-sz),(cx+sz,cy),(cx,cy+sz),(cx-sz,cy)]
    pygame.draw.polygon(screen,fill,pts); pygame.draw.polygon(screen,outline,pts,1)

# ── Main loop ─────────────────────────────────────────────────────────────────
prev_time = time.time()
running   = True

while running:
    now = time.time()
    dt  = min(now - prev_time, 0.05)   # cap at 50ms so a lag spike can't teleport
    prev_time = now

    frame_count+=1; clock.tick(60); screen.fill((20,20,30))

    camera_poll_counter+=1
    if camera_poll_counter>=CAMERA_POLL_EVERY:
        camera_poll_counter=0; update_camera_feed()
        if camera_person_detected and CAMERA_TARGET_AID not in danger_astronaut_ids:
            danger_astronaut_ids.add(CAMERA_TARGET_AID); path_dirty=True

    # IMU astronaut update with real delta-time
    apply_imu_to_astronaut(IMU_ASTRO_ID, dt)

    for gx in range(0,CENTER_X*2,SCALE): pygame.draw.line(screen,(40,40,50),(gx,0),(gx,HEIGHT))
    for gy in range(0,HEIGHT,SCALE):     pygame.draw.line(screen,(40,40,50),(0,gy),(CENTER_X*2,gy))

    for event in pygame.event.get():
        if event.type==pygame.QUIT: running=False
        if event.type==pygame.KEYDOWN:
            if   event.key==pygame.K_1: toggle_danger(1)
            elif event.key==pygame.K_2: toggle_danger(2)
            elif event.key==pygame.K_3: toggle_danger(3)
            elif event.key==pygame.K_r: reset_all()
            elif event.key==pygame.K_q: checkpoints.append((astronauts[1]["x"],astronauts[1]["y"],1)); path_dirty=True
            elif event.key==pygame.K_u: checkpoints.append((astronauts[2]["x"],astronauts[2]["y"],2)); path_dirty=True
            elif event.key==pygame.K_b: checkpoints.append((astronauts[3]["x"],astronauts[3]["y"],3)); path_dirty=True
            elif event.key==pygame.K_DELETE: checkpoints.clear(); path_dirty=True

    keys=pygame.key.get_pressed()
    if keys[pygame.K_j]: turn_l(2)
    if keys[pygame.K_l]: turn_r(2)
    if keys[pygame.K_i]: move_fwd(2)
    if keys[pygame.K_k]: move_bwd(2)
    if keys[pygame.K_f]: turn_l(3)
    if keys[pygame.K_h]: turn_r(3)
    if keys[pygame.K_t]: move_fwd(3)
    if keys[pygame.K_g]: move_bwd(3)

    update_assignments()
    if path_dirty or (helper_assignments and frame_count%PATH_RECOMPUTE_FREQ==0):
        build_and_solve(); path_dirty=False

    # Trails
    for aid,a in astronauts.items():
        p=a["path"]
        if len(p)<2: continue
        base=PATH_BASE_COLOR[aid]; total=len(p); pts=[w2s(px,py) for px,py in p]
        for i in range(1,total):
            t=i/total; c=tuple(int(ch*(0.15+0.85*t)) for ch in base)
            pygame.draw.line(screen,c,pts[i-1],pts[i],1)

    # Checkpoints
    for wx,wy,owner in checkpoints:
        px,py=w2s(wx,wy); col=CP_COLOR.get(owner,(255,255,100))
        draw_diamond(px,py,8,col,(255,255,255))
        screen.blit(sm_font.render(f"CP{owner}",True,(10,10,10)),(px-9,py-7))

    # Paths
    for did,sp in shortest_paths.items():
        if len(sp)<2: continue
        bset=set()
        for p1,p2 in bridge_segments.get(did,[]): bset.add((p1,p2)); bset.add((p2,p1))
        spts=[w2s(p[0],p[1]) for p in sp]
        for i in range(1,len(spts)):
            is_b=(sp[i-1],sp[i]) in bset
            pygame.draw.line(screen,(255,160,0) if is_b else (0,210,215),spts[i-1],spts[i],2)
            if is_b:
                mx=(spts[i-1][0]+spts[i][0])//2; my=(spts[i-1][1]+spts[i][1])//2
                pygame.draw.circle(screen,(255,200,80),(mx,my),3)
        for pt in spts[1:-1]: pygame.draw.circle(screen,(0,180,200),pt,3)
        mw=sp[len(sp)//2]; mx,my=w2s(mw[0],mw[1])
        dl=sm_font.render(f"{path_length(sp):.2f}m ({len(sp)} pts)",True,(0,220,220))
        bg=dl.get_rect(topleft=(mx+6,my-10)); pygame.draw.rect(screen,(10,10,20),bg.inflate(4,2))
        screen.blit(dl,(mx+6,my-10))

    # Hub
    pygame.draw.circle(screen,(255,255,0),(CENTER_X,CENTER_Y),12)
    screen.blit(font.render("BASE (0,0)",True,(255,255,0)),(CENTER_X+15,CENTER_Y-10))

    # Astronauts
    for aid,a in astronauts.items():
        px=int(CENTER_X+a["x"]*SCALE); py=int(CENTER_Y-a["y"]*SCALE)
        col=status_color(a["status"]); hd=dist_from_hub(aid)
        pygame.draw.line(screen,(90,90,120),(CENTER_X,CENTER_Y),(px,py),1)
        pygame.draw.circle(screen,col,(px,py),14)
        hx=px+int(24*math.cos(math.radians(a["heading"])))
        hy=py-int(24*math.sin(math.radians(a["heading"])))
        pygame.draw.line(screen,(255,255,255),(px,py),(hx,hy),2)
        lbl=(f"A{aid} | {a['status']} | x={a['x']:.2f}, y={a['y']:.2f} | "
             f"heading={a['heading']:.1f}° | dist={hd:.2f}m")
        screen.blit(font.render(lbl,True,(255,255,255)),(px+18,py-10))

    # HUD
    yt=20
    if hub_alert: screen.blit(font.render("HUB ALERT: Danger detected",True,(255,80,80)),(20,yt)); yt+=28
    if transmit_signal: screen.blit(font.render("TRANSMIT STATE: All in danger",True,(255,180,50)),(20,yt)); yt+=28
    if helper_assignments:
        for did,sid in helper_assignments.items():
            sp=shortest_paths.get(did,[]); nb=len(bridge_segments.get(did,[]))
            info=f"  ({path_length(sp):.2f}m, {nb} bridge{'s' if nb!=1 else ''})" if sp else ""
            screen.blit(font.render(f"A{sid} -> A{did}{info}",True,(50,200,255)),(20,yt)); yt+=28
    elif danger_astronaut_ids and not transmit_signal:
        screen.blit(font.render("No helper available",True,(255,180,50)),(20,yt)); yt+=28
    if checkpoints:
        screen.blit(font.render(f"Checkpoints: {len(checkpoints)}  [DEL=clear]",True,(220,220,80)),(20,yt)); yt+=28

    # IMU debug bar
    with imu_lock:
        conn=imu_data["connected"]
    imu_col=(80,255,80) if conn else (255,80,80)
    spd=math.hypot(vel_x,vel_y)
    screen.blit(font.render(
        f"IMU A1 | {'LIVE' if conn else 'NO SIGNAL'} | "
        f"pitch={f_angX:.1f}°  roll={f_angY:.1f}° | "
        f"vel=({vel_x:.2f},{vel_y:.2f}) m/s  speed={spd:.2f} m/s",
        True,imu_col),(20,HEIGHT-55))
    screen.blit(font.render(
        f"MEDIAN_N={MEDIAN_N}  EMA_α={EMA_ALPHA}  deadband=±{DEADBAND}°  "
        f"friction={FRICTION}  max_speed={MAX_SPEED}m/s",
        True,(140,140,180)),(20,HEIGHT-30))

    legend=[
        "A1: tilt board to move (velocity control — flat=stop, tilt more=faster)",
        "A2: I/K move  J/L turn  U=checkpoint     A3: T/G move  F/H turn  B=checkpoint",
        "1/2/3 toggle danger | R reset | DEL clear checkpoints | Q=A1 checkpoint",
    ]
    for i,line in enumerate(legend):
        screen.blit(font.render(line,True,(180,180,180)),(20,HEIGHT-115+i*25))

    # Camera panel
    if CAMERA_TARGET_AID in danger_astronaut_ids:
        px2,py2,pw,ph=1160,20,320,240
        pygame.draw.rect(screen,(30,30,40),(px2-10,py2-10,pw+20,ph+70))
        pygame.draw.rect(screen,(120,120,140),(px2-10,py2-10,pw+20,ph+70),2)
        screen.blit(font.render("Live Camera Feed",True,(255,255,255)),(px2,py2-35))
        screen.blit(sm_font.render(f"Person: {camera_person_detected} | Count: {camera_person_count}",
                                   True,(255,200,120)),(px2,py2+ph+10))
        if camera_surface:
            screen.blit(pygame.transform.scale(camera_surface,(pw,ph)),(px2,py2))
        else:
            pygame.draw.rect(screen,(60,60,70),(px2,py2,pw,ph))
            screen.blit(sm_font.render("No camera frame",True,(200,200,200)),(px2+100,py2+110))

    pygame.display.flip()

pygame.quit()