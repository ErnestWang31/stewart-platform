#!/usr/bin/env python3
"""
stewart_plate_sim.py

Top-down 2D Pygame simulation of a ping-pong ball on a circular plate (R=0.15 m)
with 3 actuators (120 degrees apart). Two PID controllers (X -> roll, Y -> pitch)
compute desired tilt angles which are translated to motor height targets. Includes:
- motor first-order dynamics (tau_m)
- camera sampling & delay + Gaussian noise
- derivative filtering and conditional integral anti-windup
- click-to-drop ball, pause/reset, on-screen HUD and simple keyboard tuning

Usage:
    python3 stewart_plate_sim.py

Keyboard convenience:
    Click on plate to drop ball (simulation starts)
    SPACE : pause / resume
    r     : reset (plate flat, ball returns to center, controllers reset)
    q/a   : Kp_x up/down
    w/s   : Kd_x up/down
    e/d   : Ki_x up/down
    u/j   : Kp_y up/down
    i/k   : Kd_y up/down
    o/l   : Ki_y up/down
    +/-   : increase/decrease camera delay by 5ms
    ESC   : quit

Author: ChatGPT (adapt for your project)
"""

import pygame
import numpy as np
from collections import deque
import math
import sys
import time

# -------------------------------
# Simulation parameters (physical)
# -------------------------------
# Ball (standard orange ping-pong ball)
m = 0.0027               # mass (kg)
r_ball = 0.02            # radius (m)
alpha = 0.4              # rotational inertia factor (I/(m r^2)) approx for hollow/shell
g = 9.81                 # gravity (m/s^2)
# viscous-like rolling/friction damping (tunable)
b_trans = 0.02           # (kg/s) translational damping

# Plate geometry
R_plate = 0.15           # plate radius in meters (0.15 m requested)
attachment_angles_deg = np.array([0.0, 120.0, 240.0])
attachment_angles = np.deg2rad(attachment_angles_deg)
attachment_points = np.column_stack((R_plate * np.cos(attachment_angles),
                                     R_plate * np.sin(attachment_angles)))

# Motor dynamics
# tau_m = 0.06             # motor/actuator time constant (s) - first order lag for heights
h_limit = 0.03           # motor height limit in meters (±3 cm), reasonable safe range
tau_m = 0.35             # slower motor time constant
max_dh_dt = 0.035        # m/s, maximum actuator height speed
motor_jitter_sigma = 0.0004  # m, small random jitter per step
deadband = 0.0005        # m, actuator deadband/backlash

# Simulation timing
dt = 0.001               # physics integrator timestep (s) - high frequency
control_rate = 30.0      # control (camera) update rate (Hz)
control_dt = 1.0 / control_rate
camera_delay = 0.03      # seconds of camera+processing delay (will be adjustable)
noise_sigma = 0.0015     # measurement noise (m), pixel-to-meter mapping assumed

# Conversion helpers
deg2rad = np.pi / 180.0
rad2deg = 180.0 / np.pi

# -------------------------------
# Default PID gains and filters
# -------------------------------
# Gains are on scaled-error domain (error scaled by 10 like your earlier code)
ERROR_SCALE = 10.0

# X axis PID (controls roll; moves ball in X)
pid_x = {
    'Kp': 2.2,
    'Ki': 0.25,
    'Kd': 1.3,
    'filter_alpha': 0.85,   # measurement EMA alpha
    'd_filter_alpha': 0.18, # derivative low-pass smoothing (0..1)
    'integral_threshold_frac': 0.6 # fraction of scaled units * 100 as threshold
}

# Y axis PID (controls pitch; moves ball in Y)
pid_y = {
    'Kp': 2.2,
    'Ki': 0.25,
    'Kd': 1.3,
    'filter_alpha': 0.85,
    'd_filter_alpha': 0.18,
    'integral_threshold_frac': 0.6
}

# Output angle limits (degrees)
angle_limit_deg = 15.0

# -------------------------------
# Pygame visualization parameters
# -------------------------------
WINDOW_W = 1000
WINDOW_H = 700
MARGIN = 50
PLATE_PIXELS = 500           # diameter in pixels for the plate drawing (scaled)
SCALE = PLATE_PIXELS / (2.0 * R_plate)  # px per meter (for top-down mapping)
PLATE_CENTER = (MARGIN + PLATE_PIXELS // 2, WINDOW_H // 2)

BALL_PIXEL_RADIUS = max(6, int(r_ball * SCALE))  # show ball size scaled

FONT_SIZE = 16

# Colors
BG_COLOR = (30, 30, 30)
PLATE_COLOR = (70, 130, 180)     # bluish plate
PLATE_EDGE_COLOR = (200, 200, 200)
BALL_COLOR = (230, 120, 30)      # orange ping-pong ball
ATTACH_COLOR = (200, 50, 50)
TEXT_COLOR = (240, 240, 240)
AXIS_COLOR = (180, 180, 60)
MOTOR_COLOR = (100, 220, 100)

# -------------------------------
# Utility / helper functions
# -------------------------------
def world_to_screen(pos):
    """Convert (x,y) in meters centered at plate center to screen coordinates."""
    cx, cy = PLATE_CENTER
    sx = cx + pos[0] * SCALE
    sy = cy - pos[1] * SCALE  # y up in world -> screen y down
    return int(sx), int(sy)

def screen_to_world(screen_pos):
    cx, cy = PLATE_CENTER
    sx, sy = screen_pos
    x = (sx - cx) / SCALE
    y = (cy - sy) / SCALE
    return np.array([x, y], dtype=float)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# -------------------------------
# Simple PID classes (improved)
# -------------------------------
class PID1DImproved:
    def __init__(self, Kp=2.2, Ki=0.25, Kd=1.3, filter_alpha=0.85, d_filter_alpha=0.18, integral_threshold_frac=0.6, out_limit=angle_limit_deg):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.filter_alpha = filter_alpha
        self.d_filter_alpha = d_filter_alpha
        self.integral_threshold_frac = integral_threshold_frac
        self.out_limit = out_limit

        # state
        self.integral = 0.0
        self.prev_pos_filtered = None
        self.d_filtered = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_pos_filtered = None
        self.d_filtered = 0.0

    def update(self, measured_pos, dt_control):
        # measured_pos is position (m), setpoint is 0.0 (center)
        if self.prev_pos_filtered is None:
            pos_filtered = measured_pos
        else:
            pos_filtered = self.filter_alpha * measured_pos + (1.0 - self.filter_alpha) * self.prev_pos_filtered

        error = - pos_filtered  # setpoint = 0
        error_scaled = error * ERROR_SCALE

        # Proportional
        P = self.Kp * error_scaled

        # Derivative on measurement (filtered)
        if self.prev_pos_filtered is None:
            raw_d = 0.0
        else:
            raw_d = (pos_filtered - self.prev_pos_filtered) / max(dt_control, 1e-9)
        self.d_filtered = self.d_filter_alpha * raw_d + (1.0 - self.d_filter_alpha) * self.d_filtered
        D = self.Kd * (- self.d_filtered * ERROR_SCALE)  # derivative of error is -d(pos)/dt

        # Conditional integration (anti-windup)
        if abs(error_scaled) < self.integral_threshold_frac * 100.0:
            self.integral += error_scaled * dt_control

        if self.Ki > 0.0:
            max_int = self.out_limit / max(self.Ki, 1e-9)
            self.integral = clamp(self.integral, -max_int, max_int)
            I = self.Ki * self.integral
        else:
            I = 0.0

        out = P + I + D
        out = clamp(out, -self.out_limit, self.out_limit)

        # update state
        self.prev_pos_filtered = pos_filtered
        return out  # degrees (we will convert to radians later)

# -------------------------------
# Plane / geometry utilities
# -------------------------------
def plane_from_three_points(p1, p2, p3):
    # p = (x,y,z)
    A = np.array([[p1[0], p1[1], 1.0],
                  [p2[0], p2[1], 1.0],
                  [p3[0], p3[1], 1.0]], dtype=float)
    z = np.array([p1[2], p2[2], p3[2]], dtype=float)
    a, b, c = np.linalg.solve(A, z)
    return a, b, c  # z = a x + b y + c

# For small angles: plane z(x,y) = theta_y * x - theta_x * y + c
def tilt_to_motor_targets(theta_x_rad, theta_y_rad, c_offset=0.0):
    """Given desired small-angle tilts (radians): theta_x (roll), theta_y (pitch)
       return motor height targets for each attachment point (meters)."""
    # z = theta_y * x - theta_x * y + c
    h_targets = np.zeros(3)
    for i in range(3):
        xi, yi = attachment_points[i]
        h_targets[i] = theta_y_rad * xi - theta_x_rad * yi + c_offset
    return h_targets

def motor_heights_to_tilt(h):
    """Given motor heights h (3,), compute plane and map to actual theta_x, theta_y (radians)."""
    p1 = (attachment_points[0,0], attachment_points[0,1], h[0])
    p2 = (attachment_points[1,0], attachment_points[1,1], h[1])
    p3 = (attachment_points[2,0], attachment_points[2,1], h[2])
    a, b, c = plane_from_three_points(p1, p2, p3)
    # using z = a x + b y + c -> small-angle mapping theta_y = a, theta_x = -b
    theta_y = a
    theta_x = -b
    return theta_x, theta_y, c

# -------------------------------
# Simulation state and initialization
# -------------------------------
class Simulation:
    def __init__(self):
        # ball state (world coordinates, center at plate center)
        self.p = np.array([0.0, 0.0], dtype=float)   # position (m)
        self.v = np.array([0.0, 0.0], dtype=float)   # velocity (m/s)

        # motor heights & targets
        self.h = np.zeros(3, dtype=float)            # current heights (m)
        self.h_target = np.zeros(3, dtype=float)     # targets commanded by controller

        # controllers
        self.pid_x = PID1DImproved(**{k: pid_x[k] for k in ('Kp','Ki','Kd','filter_alpha','d_filter_alpha','integral_threshold_frac')})
        self.pid_y = PID1DImproved(**{k: pid_y[k] for k in ('Kp','Ki','Kd','filter_alpha','d_filter_alpha','integral_threshold_frac')})

        # control state
        self.control_time_acc = 0.0
        self.next_control_time = 0.0

        # camera delay buffer (stores past measured positions)
        delay_steps = int(round(camera_delay / dt))
        self.delay_steps = max(1, delay_steps)
        self.meas_buffer = deque(maxlen=self.delay_steps + 1)
        for _ in range(self.delay_steps + 1):
            self.meas_buffer.append(np.array([0.0, 0.0], dtype=float))

        # flags
        self.running = False
        self.paused = False

        # controller outputs (deg)
        self.theta_cmd_deg = np.array([0.0, 0.0], dtype=float)   # [theta_x_deg, theta_y_deg]
        self.theta_act = np.array([0.0, 0.0], dtype=float)       # actual measured tilts (rad)

        # logging short trail
        self.trail = deque(maxlen=8000)

    def reset(self):
        self.__init__()

    def set_ball(self, pos_world):
        # set ball position to clicked pos if inside plate; zero velocity
        r = np.linalg.norm(pos_world)
        if r <= R_plate - r_ball * 0.99:
            self.p = pos_world.copy()
        else:
            # project to inside rim slightly
            self.p = pos_world * ((R_plate - r_ball*1.01) / r)
        self.v = np.zeros(2, dtype=float)
        # clear buffers and reset controllers
        self.meas_buffer = deque([self.p.copy() for _ in range(self.delay_steps + 1)], maxlen=self.delay_steps + 1)
        self.pid_x.reset(); self.pid_y.reset()
        self.h = np.zeros(3, dtype=float)
        self.h_target = np.zeros(3, dtype=float)
        self.trail.clear()
        self.running = True

    def step(self, dt_local):
        if self.paused or (not self.running):
            return

        # 1) motors: approach h_target (first-order)
        # semi-implicit Euler for motor heights
        # for i in range(3):
        #     dhdt = (self.h_target[i] - self.h[i]) / tau_m
        #     self.h[i] += dhdt * dt_local
        #     # clamp heights to prevent runaway
        #     self.h[i] = clamp(self.h[i], -h_limit, h_limit)

        for i in range(3):
            # desired change if purely first-order:
            dhdt_first_order = (self.h_target[i] - self.h[i]) / tau_m
            # clamp rate
            dhdt_clamped = clamp(dhdt_first_order, -max_dh_dt, max_dh_dt)
            # apply deadband: ignore very small commands
            if abs(self.h_target[i] - self.h[i]) < deadband:
                dh = 0.0
            else:
                dh = dhdt_clamped * dt_local
            # add small jitter to actual motor (simulate gear/play)
            dh += np.random.randn() * motor_jitter_sigma * math.sqrt(dt_local*1000.0)  # scale jitter with dt
            self.h[i] += dh
            # clamp heights absolutely
            self.h[i] = clamp(self.h[i], -h_limit, h_limit)

        # 2) compute actual tilt from current motor heights
        theta_x_act, theta_y_act, c = motor_heights_to_tilt(self.h)
        self.theta_act = np.array([theta_x_act, theta_y_act], dtype=float)

        # 3) ball dynamics: vector form with small-angle approx
        # acceleration = (g/(1+alpha)) * [theta_x, theta_y] - (b/m) * v
        accel = (g / (1.0 + alpha)) * np.array([theta_x_act, theta_y_act]) - (b_trans / m) * self.v
        # integrate (semi-implicit Euler)
        self.v += accel * dt_local
        self.p += self.v * dt_local

        # enforce plate boundary (project inside)
        r = np.linalg.norm(self.p)
        max_r = R_plate - r_ball * 0.999
        if r > max_r:
            # simple collision: project back to allowed radius and set radial velocity to 0
            self.p = self.p * (max_r / r)
            # kill radial component of velocity
            radial = (self.p / (np.linalg.norm(self.p) + 1e-12))
            v_radial = np.dot(self.v, radial)
            self.v -= v_radial * radial * 1.0  # damping bounce (no bounce in this simple model)

        # 4) measurement buffer (delay) - push current true position, controller reads delayed & noisy later
        self.meas_buffer.append(self.p.copy())

        # 5) control update at control_rate (use a timing accumulator approach)
        self.control_time_acc += dt_local
        while self.control_time_acc >= control_dt:
            self.control_time_acc -= control_dt
            # read delayed measurement (oldest available)
            meas = self.meas_buffer[0].copy()
            meas_noisy = meas + np.random.randn(2) * noise_sigma

            # call controllers (they return degrees)
            out_x_deg = self.pid_x.update(meas_noisy[0], control_dt)
            out_y_deg = self.pid_y.update(meas_noisy[1], control_dt)

            # store command (deg)
            self.theta_cmd_deg = np.array([out_x_deg, out_y_deg], dtype=float)

            # convert to radians and compute motor targets
            theta_x_cmd_rad = out_x_deg * deg2rad
            theta_y_cmd_rad = out_y_deg * deg2rad
            self.h_target = tilt_to_motor_targets(theta_x_cmd_rad, theta_y_cmd_rad, c_offset=0.0)
            # clamp h_target to motor range (safety)
            for i in range(3):
                self.h_target[i] = clamp(self.h_target[i], -h_limit, h_limit)

        # log trail
        self.trail.append(self.p.copy())

# -------------------------------
# Pygame drawing utilities
# -------------------------------
def draw_plate(surface):
    # draw plate circle
    cx, cy = PLATE_CENTER
    radius_px = int(R_plate * SCALE)
    pygame.draw.circle(surface, PLATE_COLOR, (cx, cy), radius_px)
    pygame.draw.circle(surface, PLATE_EDGE_COLOR, (cx, cy), radius_px, 3)
    # draw attachment points
    for i in range(3):
        pos = world_to_screen(attachment_points[i])
        pygame.draw.circle(surface, ATTACH_COLOR, pos, 8)
        # draw small label
        label = font.render(f"m{i+1}", True, TEXT_COLOR)
        surface.blit(label, (pos[0] + 6, pos[1] - 6))

def draw_axes(surface, sim):
    # draw roll/pitch axes as arrows on plate center; length scaled for visibility
    cx, cy = PLATE_CENTER
    arrow_len = int(0.04 * SCALE)  # show small arrows representing tilt direction
    # roll (theta_x): tilt about plate y-axis -> affects motion in x; draw arrow left/right
    theta_x = sim.theta_act[0]
    dx = int((theta_x * rad2deg / angle_limit_deg) * arrow_len)
    pygame.draw.line(surface, AXIS_COLOR, (cx - arrow_len, cy + 60), (cx - arrow_len + dx*2, cy + 60), 4)
    # pitch (theta_y): tilt about plate x-axis -> affects motion in y; draw arrow up/down
    theta_y = sim.theta_act[1]
    dy = int((theta_y * rad2deg / angle_limit_deg) * arrow_len)
    pygame.draw.line(surface, AXIS_COLOR, (cx + arrow_len, cy + 60), (cx + arrow_len, cy + 60 - dy*2), 4)
    # texts
    txt1 = small_font.render(f"theta_x_act: {theta_x*rad2deg:6.3f} deg", True, TEXT_COLOR)
    txt2 = small_font.render(f"theta_y_act: {theta_y*rad2deg:6.3f} deg", True, TEXT_COLOR)
    surface.blit(txt1, (cx - PLATE_PIXELS//2, cy + PLATE_PIXELS//2 + 10))
    surface.blit(txt2, (cx - PLATE_PIXELS//2, cy + PLATE_PIXELS//2 + 30))

def draw_motors(surface, sim):
    # render motor current targets and actual heights near attachment points
    for i in range(3):
        pos = world_to_screen(attachment_points[i])
        # draw vertical indicator: map height to color/length
        ht = sim.h_target[i]
        h_actual = sim.h[i]
        # convert height (m) to pixel length (small) for visualization
        len_px = int(ht * SCALE * 200)  # scale factor chosen for visibility
        len_px_act = int(h_actual * SCALE * 200)
        # draw target (thin) and actual (thicker)
        pygame.draw.line(surface, MOTOR_COLOR, (pos[0], pos[1]), (pos[0], pos[1] - len_px), 3)
        pygame.draw.line(surface, (160, 160, 160), (pos[0]+6, pos[1]), (pos[0]+6, pos[1] - len_px_act), 6)

def draw_ball(surface, sim):
    # draw ball at position p
    pos_px = world_to_screen(sim.p)
    pygame.draw.circle(surface, BALL_COLOR, pos_px, BALL_PIXEL_RADIUS)
    # draw trail
    pts = [world_to_screen(p) for p in sim.trail]
    if len(pts) > 1:
        pygame.draw.lines(surface, (255, 220, 180), False, pts, 2)

def draw_hud(surface, sim):
    # left info
    x, y = 10, 10
    def put(s, dy=0):
        nonlocal x, y
        txt = font.render(s, True, TEXT_COLOR)
        surface.blit(txt, (x, y))
        y += FONT_SIZE + 2
    put("Controls: click plate to drop ball; SPACE pause; r reset; ESC quit")
    put("PID tuning keys (X): q/a Kp up/down | w/s Kd up/down | e/d Ki up/down")
    put("PID tuning keys (Y): u/j Kp up/down | i/k Kd up/down | o/l Ki up/down")
    put(f"Camera delay +/- : +/- keys (ms). Current: {camera_delay*1000:.0f} ms")
    y += 6
    put("---- X axis PID (roll -> x) ----")
    put(f"Kp_x: {sim.pid_x.Kp:6.3f}   Kd_x: {sim.pid_x.Kd:6.3f}   Ki_x: {sim.pid_x.Ki:6.3f}")
    put("---- Y axis PID (pitch -> y) ----")
    put(f"Kp_y: {sim.pid_y.Kp:6.3f}   Kd_y: {sim.pid_y.Kd:6.3f}   Ki_y: {sim.pid_y.Ki:6.3f}")
    y += 6
    put("---- Current state ----")
    put(f"Ball pos: x={sim.p[0]:6.3f} m  y={sim.p[1]:6.3f} m")
    put(f"Ball vel: vx={sim.v[0]:6.3f} m/s  vy={sim.v[1]:6.3f} m/s")
    put(f"Theta_cmd (deg): x={sim.theta_cmd_deg[0]:6.3f}  y={sim.theta_cmd_deg[1]:6.3f}")
    put(f"Motor heights (m): {sim.h[0]:6.3f}, {sim.h[1]:6.3f}, {sim.h[2]:6.3f}")

# -------------------------------
# Main loop
# -------------------------------
def main():
    global camera_delay
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("3-motor Stewart Plate - 2D Top-Down Simulation")
    clock = pygame.time.Clock()
    global font, small_font
    font = pygame.font.SysFont("Arial", FONT_SIZE)
    small_font = pygame.font.SysFont("Arial", 14)

    sim = Simulation()

    # initial draw once
    screen.fill(BG_COLOR)
    draw_plate(screen)
    pygame.display.flip()

    paused = False

    # main event loop
    running = True
    last_time = time.time()
    while running:
        now = time.time()
        # compute real dt to step simulation multiple physics steps if needed
        frame_dt = now - last_time
        last_time = now
        # clamp frame_dt to reasonable value
        frame_dt = clamp(frame_dt, 0.0, 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                    break
                elif event.key == pygame.K_SPACE:
                    sim.paused = not sim.paused
                elif event.key == pygame.K_r:
                    sim.reset()
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    camera_delay = clamp(camera_delay + 0.005, 0.0, 0.2)
                    # resize buffer
                    sim.delay_steps = int(round(camera_delay / dt))
                    sim.meas_buffer = deque([sim.p.copy() for _ in range(sim.delay_steps + 1)], maxlen=sim.delay_steps + 1)
                elif event.key == pygame.K_MINUS:
                    camera_delay = clamp(camera_delay - 0.005, 0.0, 0.2)
                    sim.delay_steps = int(round(camera_delay / dt))
                    sim.meas_buffer = deque([sim.p.copy() for _ in range(sim.delay_steps + 1)], maxlen=sim.delay_steps + 1)

                # PID X tuning q/a (Kp up/down), w/s (Kd up/down), e/d (Ki up/down)
                elif event.key == pygame.K_q:
                    sim.pid_x.Kp += 0.1
                elif event.key == pygame.K_a:
                    sim.pid_x.Kp = max(0.0, sim.pid_x.Kp - 0.1)
                elif event.key == pygame.K_w:
                    sim.pid_x.Kd += 0.1
                elif event.key == pygame.K_s:
                    sim.pid_x.Kd = max(0.0, sim.pid_x.Kd - 0.1)
                elif event.key == pygame.K_e:
                    sim.pid_x.Ki += 0.02
                elif event.key == pygame.K_d:
                    sim.pid_x.Ki = max(0.0, sim.pid_x.Ki - 0.02)

                # PID Y tuning u/j (Kp up/down), i/k (Kd up/down), o/l (Ki up/down)
                elif event.key == pygame.K_u:
                    sim.pid_y.Kp += 0.1
                elif event.key == pygame.K_j:
                    sim.pid_y.Kp = max(0.0, sim.pid_y.Kp - 0.1)
                elif event.key == pygame.K_i:
                    sim.pid_y.Kd += 0.1
                elif event.key == pygame.K_k:
                    sim.pid_y.Kd = max(0.0, sim.pid_y.Kd - 0.1)
                elif event.key == pygame.K_o:
                    sim.pid_y.Ki += 0.02
                elif event.key == pygame.K_l:
                    sim.pid_y.Ki = max(0.0, sim.pid_y.Ki - 0.02)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # left click to set ball position and start simulation
                if event.button == 1:
                    mouse_pos = event.pos
                    world_pos = screen_to_world(mouse_pos)
                    sim.set_ball(world_pos)
                # right click could toggle pause
                elif event.button == 3:
                    sim.paused = not sim.paused

        # step simulation with multiple small increments to keep stable
        # break frame_dt into multiple dt steps of size 'dt' (physics integrator)
        remain = frame_dt
        while remain > 1e-9:
            step_dt = min(dt, remain)
            sim.step(step_dt)
            remain -= step_dt

        # render
        screen.fill(BG_COLOR)
        draw_plate(screen)
        draw_axes(screen, sim)
        draw_motors(screen, sim)
        draw_ball(screen, sim)
        draw_hud(screen, sim)
        pygame.display.flip()

        # target framerate (not required), but limit to 60 FPS
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
