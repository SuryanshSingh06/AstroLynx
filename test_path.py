import pygame
from collections import deque

pygame.init()

WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Astronaut Hub Map with Trails + Shortest Path")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

CELL = 40
GRID_COLS = WIDTH // CELL
GRID_ROWS = HEIGHT // CELL

CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2

# Optional obstacles for pathfinding demo
obstacles = set()
# Example wall:
# for r in range(5, 12):
#     obstacles.add((10, r))

astronauts = {
    1: {
        "gx": 12, "gy": 8,
        "status": "safe",
        "trail_color": (255, 120, 120),
        "path": [(12, 8)]
    },
    2: {
        "gx": 15, "gy": 6,
        "status": "safe",
        "trail_color": (120, 255, 120),
        "path": [(15, 6)]
    },
    3: {
        "gx": 8, "gy": 10,
        "status": "safe",
        "trail_color": (120, 160, 255),
        "path": [(8, 10)]
    },
}

current_rescue_path = []

def grid_to_screen(gx, gy):
    x = gx * CELL + CELL // 2
    y = gy * CELL + CELL // 2
    return x, y

def in_bounds(gx, gy):
    return 0 <= gx < GRID_COLS and 0 <= gy < GRID_ROWS

def neighbors(gx, gy):
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    result = []
    for dx, dy in dirs:
        nx, ny = gx + dx, gy + dy
        if in_bounds(nx, ny) and (nx, ny) not in obstacles:
            result.append((nx, ny))
    return result

def bfs_shortest_path(start, goal):
    if start == goal:
        return [start]

    q = deque([start])
    came_from = {start: None}

    while q:
        current = q.popleft()
        if current == goal:
            break

        for nxt in neighbors(*current):
            if nxt not in came_from:
                came_from[nxt] = current
                q.append(nxt)

    if goal not in came_from:
        return None

    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path

def move_astronaut(astro_id, dx, dy):
    a = astronauts[astro_id]
    nx, ny = a["gx"] + dx, a["gy"] + dy

    if not in_bounds(nx, ny):
        return
    if (nx, ny) in obstacles:
        return

    a["gx"], a["gy"] = nx, ny

    # Keep track of trail
    if a["path"][-1] != (nx, ny):
        a["path"].append((nx, ny))

def reset_statuses():
    global current_rescue_path
    for aid in astronauts:
        astronauts[aid]["status"] = "safe"
    current_rescue_path = []

def clear_paths():
    for aid, a in astronauts.items():
        a["path"] = [(a["gx"], a["gy"])]

def set_danger(astro_id):
    global current_rescue_path
    reset_statuses()
    astronauts[astro_id]["status"] = "danger"

    danger_pos = (astronauts[astro_id]["gx"], astronauts[astro_id]["gy"])

    best_helper = None
    best_path = None

    for aid, a in astronauts.items():
        if aid == astro_id:
            continue

        helper_pos = (a["gx"], a["gy"])
        path = bfs_shortest_path(helper_pos, danger_pos)

        if path is None:
            continue

        if best_path is None or len(path) < len(best_path):
            best_path = path
            best_helper = aid

    if best_helper is not None:
        astronauts[best_helper]["status"] = "helper"
        current_rescue_path = best_path
    else:
        current_rescue_path = []

def color_for_status(status):
    if status == "safe":
        return (0, 200, 0)
    elif status == "danger":
        return (220, 50, 50)
    elif status == "helper":
        return (50, 180, 255)
    return (200, 200, 200)

running = True
while running:
    screen.fill((20, 20, 30))

    # Draw grid
    for x in range(0, WIDTH, CELL):
        pygame.draw.line(screen, (45, 45, 60), (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, CELL):
        pygame.draw.line(screen, (45, 45, 60), (0, y), (WIDTH, y))

    # Draw obstacles
    for ox, oy in obstacles:
        rect = pygame.Rect(ox * CELL, oy * CELL, CELL, CELL)
        pygame.draw.rect(screen, (80, 80, 80), rect)

    # Draw base at screen center
    pygame.draw.circle(screen, (255, 255, 0), (CENTER_X, CENTER_Y), 12)
    hub_text = font.render("BASE", True, (255, 255, 0))
    screen.blit(hub_text, (CENTER_X + 15, CENTER_Y - 10))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            # Astronaut 1: WASD
            if event.key == pygame.K_w:
                move_astronaut(1, 0, -1)
            elif event.key == pygame.K_s:
                move_astronaut(1, 0, 1)
            elif event.key == pygame.K_a:
                move_astronaut(1, -1, 0)
            elif event.key == pygame.K_d:
                move_astronaut(1, 1, 0)

            # Optional testing keys for astronaut 2
            elif event.key == pygame.K_i:
                move_astronaut(2, 0, -1)
            elif event.key == pygame.K_k:
                move_astronaut(2, 0, 1)
            elif event.key == pygame.K_j:
                move_astronaut(2, -1, 0)
            elif event.key == pygame.K_l:
                move_astronaut(2, 1, 0)

            # Optional testing keys for astronaut 3
            elif event.key == pygame.K_t:
                move_astronaut(3, 0, -1)
            elif event.key == pygame.K_g:
                move_astronaut(3, 0, 1)
            elif event.key == pygame.K_f:
                move_astronaut(3, -1, 0)
            elif event.key == pygame.K_h:
                move_astronaut(3, 1, 0)

            elif event.key == pygame.K_1:
                set_danger(1)
            elif event.key == pygame.K_2:
                set_danger(2)
            elif event.key == pygame.K_3:
                set_danger(3)
            elif event.key == pygame.K_r:
                reset_statuses()
            elif event.key == pygame.K_c:
                clear_paths()

    # Draw astronaut trails
    for aid, a in astronauts.items():
        if len(a["path"]) >= 2:
            for i in range(1, len(a["path"])):
                x1, y1 = grid_to_screen(*a["path"][i - 1])
                x2, y2 = grid_to_screen(*a["path"][i])
                pygame.draw.line(screen, a["trail_color"], (x1, y1), (x2, y2), 4)

    # Draw rescue shortest path
    if current_rescue_path and len(current_rescue_path) >= 2:
        for i in range(1, len(current_rescue_path)):
            x1, y1 = grid_to_screen(*current_rescue_path[i - 1])
            x2, y2 = grid_to_screen(*current_rescue_path[i])
            pygame.draw.line(screen, (50, 220, 255), (x1, y1), (x2, y2), 6)

    # Draw astronauts
    for aid, a in astronauts.items():
        px, py = grid_to_screen(a["gx"], a["gy"])
        color = color_for_status(a["status"])

        pygame.draw.circle(screen, color, (px, py), 14)

        label = f"A{aid} | {a['status']} | ({a['gx']}, {a['gy']})"
        text = font.render(label, True, (255, 255, 255))
        screen.blit(text, (px + 18, py - 10))

    # Rescue message
    danger_id = None
    helper_id = None
    for aid, a in astronauts.items():
        if a["status"] == "danger":
            danger_id = aid
        elif a["status"] == "helper":
            helper_id = aid

    if danger_id and helper_id:
        msg = f"Shortest-path responder: Astronaut {helper_id} -> Assist Astronaut {danger_id}"
        msg_text = font.render(msg, True, (50, 220, 255))
        screen.blit(msg_text, (20, 20))

    instructions = [
        "A1: WASD",
        "A2: IJKL",
        "A3: TFGH",
        "1/2/3 = mark astronaut in danger",
        "R = reset statuses",
        "C = clear trails"
    ]
    for idx, line in enumerate(instructions):
        text = font.render(line, True, (200, 200, 200))
        screen.blit(text, (20, HEIGHT - 160 + idx * 25))

    pygame.display.flip()
    clock.tick(30)

pygame.quit()