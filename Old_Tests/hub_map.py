import pygame
import math

pygame.init()

WIDTH, HEIGHT = 1150, 780
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Astronaut Hub Map - Multi Danger Logic")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE = 60  # pixels per meter

# movement tuning
MOVE_SPEED = 0.04
TURN_SPEED = 2.5

astronauts = {
    1: {"x": 0.0, "y": 0.0, "heading": 0.0, "status": "safe"},
    2: {"x": 2.0, "y": 1.0, "heading": 90.0, "status": "safe"},
    3: {"x": -2.0, "y": -1.0, "heading": 45.0, "status": "safe"},
}

danger_astronaut_ids = set()
helper_assignments = {}   # danger_id -> helper_id
hub_alert = False
transmit_signal = False

def move_forward(astro_id, speed=MOVE_SPEED):
    a = astronauts[astro_id]
    angle_rad = math.radians(a["heading"])
    a["x"] += speed * math.cos(angle_rad)
    a["y"] += speed * math.sin(angle_rad)

def move_backward(astro_id, speed=MOVE_SPEED):
    a = astronauts[astro_id]
    angle_rad = math.radians(a["heading"])
    a["x"] -= speed * math.cos(angle_rad)
    a["y"] -= speed * math.sin(angle_rad)

def turn_left(astro_id, amount=TURN_SPEED):
    astronauts[astro_id]["heading"] = (astronauts[astro_id]["heading"] - amount) % 360

def turn_right(astro_id, amount=TURN_SPEED):
    astronauts[astro_id]["heading"] = (astronauts[astro_id]["heading"] + amount) % 360

def distance_between(a1, a2):
    return math.sqrt((a1["x"] - a2["x"])**2 + (a1["y"] - a2["y"])**2)

def distance_from_hub(astro_id):
    a = astronauts[astro_id]
    return math.sqrt(a["x"]**2 + a["y"]**2)

def toggle_danger(astro_id):
    if astro_id in danger_astronaut_ids:
        danger_astronaut_ids.remove(astro_id)
    else:
        danger_astronaut_ids.add(astro_id)
    update_assignments_and_alerts()

def reset_all():
    global hub_alert, transmit_signal
    danger_astronaut_ids.clear()
    helper_assignments.clear()
    hub_alert = False
    transmit_signal = False
    for aid in astronauts:
        astronauts[aid]["status"] = "safe"

def update_assignments_and_alerts():
    global helper_assignments, hub_alert, transmit_signal

    # Reset statuses first
    for aid in astronauts:
        astronauts[aid]["status"] = "safe"

    helper_assignments = {}

    # Alert is always on if anyone is in danger
    hub_alert = len(danger_astronaut_ids) > 0

    # Transmit signal if all astronauts are in danger
    transmit_signal = len(danger_astronaut_ids) == len(astronauts)

    # Mark dangers first
    for did in danger_astronaut_ids:
        astronauts[did]["status"] = "danger"

    safe_ids = [aid for aid in astronauts if aid not in danger_astronaut_ids]

    # If no safe astronauts exist, nobody can help
    if not safe_ids:
        return

    # Build all danger-safe pairs and sort by closest distance first
    pairs = []
    for did in danger_astronaut_ids:
        for sid in safe_ids:
            dist = distance_between(astronauts[did], astronauts[sid])
            pairs.append((dist, did, sid))

    pairs.sort(key=lambda x: x[0])

    assigned_dangers = set()
    assigned_helpers = set()

    # Greedy assignment: closest distance first
    for dist, did, sid in pairs:
        if did in assigned_dangers:
            continue
        if sid in assigned_helpers:
            continue

        helper_assignments[did] = sid
        assigned_dangers.add(did)
        assigned_helpers.add(sid)

    # Update helper statuses
    for did, sid in helper_assignments.items():
        astronauts[sid]["status"] = "helper"

    # Mark danger astronauts without helpers
    for did in danger_astronaut_ids:
        if did not in helper_assignments:
            astronauts[did]["status"] = "danger_unassisted"

def color_for_status(status):
    if status == "safe":
        return (0, 200, 0)
    elif status == "danger":
        return (220, 50, 50)
    elif status == "danger_unassisted":
        return (255, 140, 0)
    elif status == "helper":
        return (50, 100, 255)
    return (200, 200, 200)

running = True
while running:
    clock.tick(60)
    screen.fill((20, 20, 30))

    # grid
    for x in range(0, WIDTH, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (0, y), (WIDTH, y))

    # events
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

    # held-key controls for smooth curved motion
    keys = pygame.key.get_pressed()

    # Astronaut 1: W/A/D/S
    if keys[pygame.K_a]:
        turn_left(1)
    if keys[pygame.K_d]:
        turn_right(1)
    if keys[pygame.K_w]:
        move_forward(1)
    if keys[pygame.K_s]:
        move_backward(1)

    # Astronaut 2: I/J/L/K
    if keys[pygame.K_j]:
        turn_left(2)
    if keys[pygame.K_l]:
        turn_right(2)
    if keys[pygame.K_i]:
        move_forward(2)
    if keys[pygame.K_k]:
        move_backward(2)

    # Astronaut 3: T/F/H/G
    if keys[pygame.K_f]:
        turn_left(3)
    if keys[pygame.K_h]:
        turn_right(3)
    if keys[pygame.K_t]:
        move_forward(3)
    if keys[pygame.K_g]:
        move_backward(3)

    # Recompute assignments every frame so helpers update live
    update_assignments_and_alerts()

    # hub/base
    pygame.draw.circle(screen, (255, 255, 0), (CENTER_X, CENTER_Y), 12)
    hub_text = font.render("BASE (0,0)", True, (255, 255, 0))
    screen.blit(hub_text, (CENTER_X + 15, CENTER_Y - 10))

    # draw astronauts
    for aid, a in astronauts.items():
        px = int(CENTER_X + a["x"] * SCALE)
        py = int(CENTER_Y - a["y"] * SCALE)
        color = color_for_status(a["status"])
        hub_dist = distance_from_hub(aid)

        # line to hub
        pygame.draw.line(screen, (90, 90, 120), (CENTER_X, CENTER_Y), (px, py), 1)

        # astronaut
        pygame.draw.circle(screen, color, (px, py), 14)

        # heading line
        hx = px + int(24 * math.cos(math.radians(a["heading"])))
        hy = py - int(24 * math.sin(math.radians(a["heading"])))
        pygame.draw.line(screen, (255, 255, 255), (px, py), (hx, hy), 2)

        label = (
            f"A{aid} | {a['status']} | "
            f"x={a['x']:.2f}, y={a['y']:.2f} | "
            f"heading={a['heading']:.1f}° | "
            f"dist to hub={hub_dist:.2f}m"
        )
        text = font.render(label, True, (255, 255, 255))
        screen.blit(text, (px + 18, py - 10))

    # draw helper lines
    for did, sid in helper_assignments.items():
        d = astronauts[did]
        h = astronauts[sid]
        x1 = int(CENTER_X + d["x"] * SCALE)
        y1 = int(CENTER_Y - d["y"] * SCALE)
        x2 = int(CENTER_X + h["x"] * SCALE)
        y2 = int(CENTER_Y - h["y"] * SCALE)
        pygame.draw.line(screen, (50, 100, 255), (x1, y1), (x2, y2), 3)

    # top-left status messages
    y_text = 20

    if hub_alert:
        alert_text = font.render("HUB ALERT: Danger detected", True, (255, 80, 80))
        screen.blit(alert_text, (20, y_text))
        y_text += 28

    if transmit_signal:
        transmit_text = font.render("TRANSMIT STATE: All astronauts in danger", True, (255, 180, 50))
        screen.blit(transmit_text, (20, y_text))
        y_text += 28

    if helper_assignments:
        for did, sid in helper_assignments.items():
            msg = f"Astronaut {sid} assisting Astronaut {did}"
            msg_text = font.render(msg, True, (50, 200, 255))
            screen.blit(msg_text, (20, y_text))
            y_text += 28
    elif danger_astronaut_ids and not transmit_signal:
        no_help_text = font.render("No available helper for one or more danger astronauts", True, (255, 180, 50))
        screen.blit(no_help_text, (20, y_text))
        y_text += 28

    instructions = [
        "A1: W/S move, A/D turn",
        "A2: I/K move, J/L turn",
        "A3: T/G move, F/H turn",
        "1,2,3 = toggle danger | R = reset",
        "Closest safe astronaut helps first",
        "Alert always sent if any astronaut is in danger",
        "Transmit state when all astronauts are in danger"
    ]
    for idx, line in enumerate(instructions):
        text = font.render(line, True, (200, 200, 200))
        screen.blit(text, (20, HEIGHT - 185 + idx * 25))

    pygame.display.flip()

pygame.quit()