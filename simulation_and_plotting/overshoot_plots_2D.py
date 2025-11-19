#!/usr/bin/env python3
"""
stewart_pid_sim_2d.py

2D simulation of a ping-pong ball on a 3-actuator (120°) Stewart-like platform.
Compares a "Baseline PID" and an "Improved PID" in 2D with camera delay/noise.

Save and run:
    python3 stewart_pid_sim_2d.py

Dependencies:
    numpy, matplotlib
"""
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
import os

# ----------------- Physical params -----------------
m = 0.0027        # kg (ping-pong ball)
r_ball = 0.02     # m
alpha = 0.4       # rotational inertia factor
g = 9.81          # m/s^2
b = 0.02          # damping (kg/s)
tau_m = 0.05      # motor/platform time constant (s)

# platform geometry
R = 0.09  # radius where actuators attach (m)
angles_deg = np.array([0.0, 120.0, 240.0])
angles = np.deg2rad(angles_deg)
attachment_points = np.column_stack((R * np.cos(angles), R * np.sin(angles)))  # (x,y)

# ----------------- Simulation timing -----------------
sim_t = 8.0
dt = 0.001
time = np.arange(0.0, sim_t, dt)

control_rate = 30.0
control_dt = 1.0 / control_rate
control_delay = 0.03
noise_sigma = 0.0015

# initial ball pos (x,y)
x0 = 0.020  # 20 mm in x
y0 = -0.012 # -12 mm in y (so there's a 2D offset)

# output limits
output_limit_deg = 15.0
deg2rad = np.pi / 180.0

# ----------------- PID implementations (2D wrappers) -----------------
@dataclass
class PID1D:
    Kp: float; Ki: float; Kd: float; filter_alpha: float; error_scale: float = 10.0
    integral: float = 0.0
    prev_error_scaled: float = 0.0
    pos_filtered: float = None

    def reset(self):
        self.integral = 0.0
        self.prev_error_scaled = 0.0
        self.pos_filtered = None

    def update(self, measured_pos, dt_control):
        if self.pos_filtered is None:
            self.pos_filtered = measured_pos
        else:
            self.pos_filtered = self.filter_alpha * measured_pos + (1 - self.filter_alpha) * self.pos_filtered
        error = 0.0 - self.pos_filtered
        error_scaled = error * self.error_scale
        P = self.Kp * error_scaled
        I = 0.0
        if self.Ki > 0:
            self.integral += error_scaled * dt_control
            max_int = output_limit_deg / max(self.Ki, 1e-9)
            self.integral = np.clip(self.integral, -max_int, max_int)
            I = self.Ki * self.integral
        D = self.Kd * (error_scaled - self.prev_error_scaled) / max(dt_control, 1e-9)
        out = P + I + D
        self.prev_error_scaled = error_scaled
        return out

@dataclass
class PID1DImproved:
    Kp: float; Ki: float; Kd: float; filter_alpha: float; d_filter_alpha: float = 0.2; error_scale: float = 10.0
    integral_threshold_frac: float = 0.6
    integral: float = 0.0
    prev_pos_filtered: float = None
    d_filtered: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_pos_filtered = None
        self.d_filtered = 0.0

    def update(self, measured_pos, dt_control):
        if self.prev_pos_filtered is None:
            pos_filtered = measured_pos
            self.prev_pos_filtered = pos_filtered
        else:
            pos_filtered = self.filter_alpha * measured_pos + (1 - self.filter_alpha) * self.prev_pos_filtered
        error = - pos_filtered
        error_scaled = error * self.error_scale
        P = self.Kp * error_scaled
        raw_d = (pos_filtered - self.prev_pos_filtered) / max(dt_control, 1e-9)
        self.d_filtered = self.d_filter_alpha * raw_d + (1 - self.d_filter_alpha) * self.d_filtered
        D = self.Kd * (- self.d_filtered * self.error_scale)
        if abs(error_scaled) < self.integral_threshold_frac * 100.0:
            self.integral += error_scaled * dt_control
        if self.Ki > 0:
            max_int = output_limit_deg / max(self.Ki, 1e-9)
            self.integral = np.clip(self.integral, -max_int, max_int)
            I = self.Ki * self.integral
        else:
            I = 0.0
        out = P + I + D
        self.prev_pos_filtered = pos_filtered
        return out

@dataclass
class Controller2D_Baseline:
    pid_x: PID1D = field(default_factory=lambda: PID1D(Kp=1.8, Ki=0.0, Kd=0.7, filter_alpha=0.7))
    pid_y: PID1D = field(default_factory=lambda: PID1D(Kp=1.8, Ki=0.0, Kd=0.7, filter_alpha=0.7))
    def reset(self):
        self.pid_x.reset(); self.pid_y.reset()
    def update(self, meas_x, meas_y, dt_control):
        out_x = self.pid_x.update(meas_x, dt_control)
        out_y = self.pid_y.update(meas_y, dt_control)
        out_x = np.clip(out_x, -output_limit_deg, output_limit_deg)
        out_y = np.clip(out_y, -output_limit_deg, output_limit_deg)
        return out_x, out_y

@dataclass
class Controller2D_Improved:
    pid_x: PID1DImproved = field(default_factory=lambda: PID1DImproved(Kp=2.2, Ki=0.25, Kd=1.3, filter_alpha=0.85, d_filter_alpha=0.18))
    pid_y: PID1DImproved = field(default_factory=lambda: PID1DImproved(Kp=2.2, Ki=0.25, Kd=1.3, filter_alpha=0.85, d_filter_alpha=0.18))
    def reset(self):
        self.pid_x.reset(); self.pid_y.reset()
    def update(self, meas_x, meas_y, dt_control):
        out_x = self.pid_x.update(meas_x, dt_control)
        out_y = self.pid_y.update(meas_y, dt_control)
        out_x = np.clip(out_x, -output_limit_deg, output_limit_deg)
        out_y = np.clip(out_y, -output_limit_deg, output_limit_deg)
        return out_x, out_y

# ----------------- Utilities -----------------
def plane_from_three_points(p1, p2, p3):
    A = np.array([[p1[0], p1[1], 1.0],
                  [p2[0], p2[1], 1.0],
                  [p3[0], p3[1], 1.0]])
    z = np.array([p1[2], p2[2], p3[2]])
    a, b, c = np.linalg.solve(A, z)
    return a, b, c

def step_motor_heights(h_current, h_target, dt):
    dhdt = (h_target - h_current) / tau_m
    return h_current + dhdt * dt

def ball_dynamics_2d(pos, vel, theta_x, theta_y, dt):
    accel = (g / (1 + alpha)) * np.array([theta_x, theta_y]) - (b / m) * vel
    vel_next = vel + accel * dt
    pos_next = pos + vel_next * dt
    return pos_next, vel_next

# ----------------- Runner -----------------
def run_sim_2d(controller2d, seed=0):
    np.random.seed(seed)
    controller2d.reset()
    pos = np.array([x0, y0], dtype=float)
    vel = np.array([0.0, 0.0], dtype=float)
    h = np.zeros(3)
    logs = { 'time':[], 'pos':[], 'vel':[], 'theta_cmd':[], 'theta_act':[], 'h':[], 'motor_cmds':[] }
    next_control_time = 0.0
    delay_steps = int(round(control_delay / dt))
    meas_buffer = [np.array([x0,y0])] * (delay_steps + 1)
    motor_targets = np.zeros(3)
    control_deg = np.zeros(2)
    for ti in time:
        for i in range(3):
            h[i] = step_motor_heights(h[i], motor_targets[i], dt)
        p1 = (attachment_points[0,0], attachment_points[0,1], h[0])
        p2 = (attachment_points[1,0], attachment_points[1,1], h[1])
        p3 = (attachment_points[2,0], attachment_points[2,1], h[2])
        a, b, c = plane_from_three_points(p1, p2, p3)
        theta_x_act = -b
        theta_y_act = a
        pos, vel = ball_dynamics_2d(pos, vel, theta_x_act, theta_y_act, dt)
        meas_buffer.append(pos.copy())
        meas = meas_buffer.pop(0)
        meas_noisy = meas + np.random.randn(2) * noise_sigma
        if ti >= next_control_time - 1e-12:
            out_x_deg, out_y_deg = controller2d.update(meas_noisy[0], meas_noisy[1], control_dt)
            theta_x_cmd = out_x_deg * deg2rad
            theta_y_cmd = out_y_deg * deg2rad
            control_deg = np.array([out_x_deg, out_y_deg])
            motor_targets = np.array([theta_y_cmd * attachment_points[i,0] - theta_x_cmd * attachment_points[i,1] for i in range(3)])
            next_control_time += control_dt
        logs['time'].append(ti)
        logs['pos'].append(pos.copy())
        logs['vel'].append(vel.copy())
        logs['theta_cmd'].append(control_deg.copy())
        logs['theta_act'].append(np.array([theta_x_act, theta_y_act]))
        logs['h'].append(h.copy())
        logs['motor_cmds'].append(motor_targets.copy())
    for k in logs:
        logs[k] = np.array(logs[k])
    return logs

def distance_to_center(pos_array):
    return np.linalg.norm(pos_array, axis=1)

def plot_results(res_b, res_i):
    out_dir = "simulation_and_plotting/sim2d_outputs"
    os.makedirs(out_dir, exist_ok=True)
    t = res_b['time']
    plt.figure(figsize=(6,6))
    plt.plot(res_b['pos'][:,0], res_b['pos'][:,1], label='Baseline')
    plt.plot(res_i['pos'][:,0], res_i['pos'][:,1], label='Improved')
    plt.scatter([0],[0], marker='x', color='k', label='center')
    plt.xlabel('X (m)'); plt.ylabel('Y (m)'); plt.title('XY trajectory'); plt.legend(); plt.grid(True)
    plt.axis('equal')
    plt.tight_layout(); plt.savefig(os.path.join(out_dir, "traj_xy.png"), dpi=200)
    dist_b = distance_to_center(res_b['pos'])
    dist_i = distance_to_center(res_i['pos'])
    plt.figure(figsize=(8,4))
    plt.plot(t, dist_b, label='Baseline')
    plt.plot(t, dist_i, label='Improved')
    plt.xlabel('Time (s)'); plt.ylabel('Distance to center (m)')
    plt.title('Distance to center over time'); plt.grid(True); plt.legend()
    plt.tight_layout(); plt.savefig(os.path.join(out_dir, "dist_time.png"), dpi=200)
    plt.figure(figsize=(10,4))
    plt.plot(t, res_b['theta_cmd'][:,0], label='Baseline theta_x (deg)')
    plt.plot(t, res_b['theta_cmd'][:,1], label='Baseline theta_y (deg)')
    plt.plot(t, res_i['theta_cmd'][:,0], '--', label='Improved theta_x (deg)')
    plt.plot(t, res_i['theta_cmd'][:,1], '--', label='Improved theta_y (deg)')
    plt.xlabel('Time (s)'); plt.ylabel('Theta commands (deg)'); plt.title('Theta command time series')
    plt.legend(); plt.grid(True); plt.tight_layout(); plt.savefig(os.path.join(out_dir, "theta_cmds.png"), dpi=200)
    plt.figure(figsize=(10,4))
    plt.plot(t, res_i['motor_cmds'][:,0], label='motor0 target (m)')
    plt.plot(t, res_i['motor_cmds'][:,1], label='motor1 target (m)')
    plt.plot(t, res_i['motor_cmds'][:,2], label='motor2 target (m)')
    plt.xlabel('Time (s)'); plt.ylabel('Motor target heights (m)'); plt.title('Improved motor targets'); plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "motor_targets_improved.png"), dpi=200)
    print("Saved plots to", os.path.abspath(out_dir))

# ----------------- Execute sims -----------------
def main():
    baseline_ctrl = Controller2D_Baseline()
    improved_ctrl = Controller2D_Improved()
    res_b = run_sim_2d(baseline_ctrl, seed=1)
    res_i = run_sim_2d(improved_ctrl, seed=1)
    dist_b = distance_to_center(res_b['pos'])
    dist_i = distance_to_center(res_i['pos'])
    initial_dist = np.linalg.norm(np.array([x0,y0]))
    min_dist_b = np.min(dist_b)
    min_dist_i = np.min(dist_i)
    overshoot_b = max(0.0, (initial_dist - min_dist_b) / initial_dist * 100.0)
    overshoot_i = max(0.0, (initial_dist - min_dist_i) / initial_dist * 100.0)
    def settling_time(dist, time_arr, pct=0.05):
        thresh = pct * initial_dist
        for i in range(len(dist)):
            if np.all(dist[i:] <= thresh):
                return time_arr[i]
        return np.nan
    settle_b = settling_time(dist_b, res_b['time'], pct=0.05)
    settle_i = settling_time(dist_i, res_i['time'], pct=0.05)
    print("2D Simulation metrics (initial dist = {:.3f} m):".format(initial_dist))
    print("Baseline: min dist = {:.4f} m, overshoot = {:.2f} %, 5% settling = {}".format(min_dist_b, overshoot_b, settle_b))
    print("Improved: min dist = {:.4f} m, overshoot = {:.2f} %, 5% settling = {}".format(min_dist_i, overshoot_i, settle_i))
    plot_results(res_b, res_i)
    try:
        plt.show()
    except:
        pass

if __name__ == "__main__":
    main()
