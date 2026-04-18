import pygame
import math

pygame.init()

WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Astronaut Hub Map")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
SCALE = 60  # pixels per meter

astronauts = {
    1: {"x": 0.0, "y": 0.0, "heading": 0, "status": "safe"},
    2: {"x": 2.0, "y": 1.0, "heading": 90, "status": "safe"},
    3: {"x": -2.0, "y": -1.0, "heading": 45, "status": "safe"},
}

def move_forward(astro_id, step=0.5):
    a = astronauts[astro_id]
    angle_rad = math.radians(a["heading"])
    a["x"] += step * math.cos(angle_rad)
    a["y"] += step * math.sin(angle_rad)

def turn(astro_id, degrees):
    astronauts[astro_id]["heading"] = (astronauts[astro_id]["heading"] + degrees) % 360

def distance(a1, a2):
    return math.sqrt((a1["x"] - a2["x"])**2 + (a1["y"] - a2["y"])**2)

def reset_statuses():
    for aid in astronauts:
        astronauts[aid]["status"] = "safe"

def set_danger(astro_id):
    reset_statuses()
    astronauts[astro_id]["status"] = "danger"

    nearest_id = None
    nearest_dist = float("inf")

    for aid, a in astronauts.items():
        if aid == astro_id:
            continue
        d = distance(astronauts[astro_id], a)
        if d < nearest_dist:
            nearest_dist = d
            nearest_id = aid

    if nearest_id is not None:
        astronauts[nearest_id]["status"] = "helper"

def color_for_status(status):
    if status == "safe":
        return (0, 200, 0)
    elif status == "danger":
        return (220, 50, 50)
    elif status == "helper":
        return (50, 100, 255)
    return (200, 200, 200)

running = True
while running:
    screen.fill((20, 20, 30))

    # Draw grid
    for x in range(0, WIDTH, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, SCALE):
        pygame.draw.line(screen, (40, 40, 50), (0, y), (WIDTH, y))

    # Draw hub/base
    pygame.draw.circle(screen, (255, 255, 0), (CENTER_X, CENTER_Y), 12)
    hub_text = font.render("BASE", True, (255, 255, 0))
    screen.blit(hub_text, (CENTER_X + 15, CENTER_Y - 10))

    # Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            # Astronaut 1 controls: W/A/D, 1 danger
            if event.key == pygame.K_w:
                move_forward(1)
            elif event.key == pygame.K_a:
                turn(1, -15)
            elif event.key == pygame.K_d:
                turn(1, 15)
            elif event.key == pygame.K_1:
                set_danger(1)

            # Astronaut 2 controls: I/J/L, 2 danger
            elif event.key == pygame.K_i:
                move_forward(2)
            elif event.key == pygame.K_j:
                turn(2, -15)
            elif event.key == pygame.K_l:
                turn(2, 15)
            elif event.key == pygame.K_2:
                set_danger(2)

            # Astronaut 3 controls: T/F/H, 3 danger
            elif event.key == pygame.K_t:
                move_forward(3)
            elif event.key == pygame.K_f:
                turn(3, -15)
            elif event.key == pygame.K_h:
                turn(3, 15)
            elif event.key == pygame.K_3:
                set_danger(3)

            elif event.key == pygame.K_r:
                reset_statuses()

    # Draw astronauts
    for aid, a in astronauts.items():
        px = int(CENTER_X + a["x"] * SCALE)
        py = int(CENTER_Y - a["y"] * SCALE)

        color = color_for_status(a["status"])
        pygame.draw.circle(screen, color, (px, py), 14)

        # heading line
        hx = px + int(20 * math.cos(math.radians(a["heading"])))
        hy = py - int(20 * math.sin(math.radians(a["heading"])))
        pygame.draw.line(screen, (255, 255, 255), (px, py), (hx, hy), 2)

        label = f"A{aid} | {a['status']} | ({a['x']:.1f}, {a['y']:.1f})"
        text = font.render(label, True, (255, 255, 255))
        screen.blit(text, (px + 18, py - 10))

    # Draw rescue line
    danger_id = None
    helper_id = None
    for aid, a in astronauts.items():
        if a["status"] == "danger":
            danger_id = aid
        elif a["status"] == "helper":
            helper_id = aid

    if danger_id and helper_id:
        d = astronauts[danger_id]
        h = astronauts[helper_id]
        x1 = int(CENTER_X + d["x"] * SCALE)
        y1 = int(CENTER_Y - d["y"] * SCALE)
        x2 = int(CENTER_X + h["x"] * SCALE)
        y2 = int(CENTER_Y - h["y"] * SCALE)
        pygame.draw.line(screen, (50, 100, 255), (x1, y1), (x2, y2), 3)

        msg = f"Nearest responder: Astronaut {helper_id} -> Assist Astronaut {danger_id}"
        msg_text = font.render(msg, True, (50, 200, 255))
        screen.blit(msg_text, (20, 20))

    instructions = [
        "A1: W/A/D + 1 danger",
        "A2: I/J/L + 2 danger",
        "A3: T/F/H + 3 danger",
        "R = reset"
    ]
    for idx, line in enumerate(instructions):
        text = font.render(line, True, (200, 200, 200))
        screen.blit(text, (20, HEIGHT - 120 + idx * 25))

    pygame.display.flip()
    clock.tick(30)

pygame.quit()