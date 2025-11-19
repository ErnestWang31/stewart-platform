#!/usr/bin/env python3
"""
stewart_pid_sim.py

1D simulation of a ping-pong ball on a tilting platform with a camera-based PID controller.
Compares a "Baseline PID" and an "Improved PID" and saves plots to the current folder.

Save as stewart_pid_sim.py and run:
    python3 stewart_pid_sim.py

Dependencies:
    numpy, matplotlib

Author: ChatGPT (adapted for your project)
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import os

# -------------------------
# Simulation / physical params
# -------------------------
m = 0.0027        # mass of ping-pong ball (kg)
r_ball = 0.02     # radius (m)
alpha = 0.4       # rotational inertia factor (I/(m r^2)) approx
g = 9.81          # gravity (m/s^2)
b = 0.02          # viscous damping estimate (kg/s)
tau_m = 0.05      # motor/platform time constant (s)
platform_radius = 0.1  # m

# Simulation timing
sim_t = 6.0        # total sim time (s)
dt = 0.001         # physics integrator time step (s)
t = np.arange(0.0, sim_t, dt)

# Camera / control
control_rate = 30.0       # Hz (camera/control loop)
control_dt = 1.0 / control_rate
control_delay = 0.03      # camera + processing delay (s)
noise_sigma = 0.0015      # measurement noise standard deviation (m)

# Initial condition (displaced ball)
x0 = 0.020  # 20 mm initial offset

# Controller output limits (degrees)
output_limit_deg = 15.0
deg2rad = np.pi / 180.0

# -------------------------
# Utility metrics
# -------------------------
def compute_overshoot_from_positive_step(signal):
    """
    For a positive initial displacement step toward zero.
    Overshoot is defined as maximum negative excursion as percentage of initial amplitude.
    """
    initial = signal[0]
    if initial <= 0:
        return 0.0, np.min(signal)
    min_val = np.min(signal)
    if min_val >= 0:
        return 0.0, min_val
    overshoot_pct = (-min_val / initial) * 100.0
    return overshoot_pct, min_val

def settling_time_percent(time, signal, pct=0.02):
    initial = abs(signal[0])
    if initial < 1e-9:
        return 0.0
    thresh = pct * initial
    for i in range(len(signal)):
        if np.all(np.abs(signal[i:]) <= thresh):
            return time[i]
    return np.nan

# -------------------------
# PID classes
# -------------------------
@dataclass
class BaselinePID:
    Kp: float = 1.8
    Ki: float = 0.0
    Kd: float = 0.7
    output_limit: float = output_limit_deg
    error_scale: float = 10.0
    filter_alpha: float = 0.7

    integral: float = 0.0
    prev_error_scaled: float = 0.0
    pos_filtered: float = None

    def reset(self):
        self.integral = 0.0
        self.prev_error_scaled = 0.0
        self.pos_filtered = None

    def update(self, measured_pos, dt_control):
        # EMA measurement filter
        if self.pos_filtered is None:
            self.pos_filtered = measured_pos
        else:
            self.pos_filtered = self.filter_alpha * measured_pos + (1 - self.filter_alpha) * self.pos_filtered

        error = 0.0 - self.pos_filtered
        error_scaled = error * self.error_scale

        P = self.Kp * error_scaled

        if self.Ki > 0.0:
            self.integral += error_scaled * dt_control
            max_int = self.output_limit / max(self.Ki, 1e-9)
            self.integral = np.clip(self.integral, -max_int, max_int)
            I = self.Ki * self.integral
        else:
            I = 0.0

        derivative = (error_scaled - self.prev_error_scaled) / max(dt_control, 1e-9)
        D = self.Kd * derivative

        out = P + I + D
        out = np.clip(out, -self.output_limit, self.output_limit)

        self.prev_error_scaled = error_scaled
        return out

@dataclass
class ImprovedPID:
    Kp: float = 2.2
    Ki: float = 0.25
    Kd: float = 1.3
    output_limit: float = output_limit_deg
    error_scale: float = 10.0
    filter_alpha: float = 0.85
    d_filter_alpha: float = 0.18
    integral_threshold: float = 0.6   # near-center threshold (fraction of 1 in scaled units)

    integral: float = 0.0
    prev_pos_filtered: float = None
    d_filtered: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_pos_filtered = None
        self.d_filtered = 0.0

    def update(self, measured_pos, dt_control):
        # EMA measurement filter
        if self.prev_pos_filtered is None:
            pos_filtered = measured_pos
            self.prev_pos_filtered = pos_filtered
        else:
            pos_filtered = self.filter_alpha * measured_pos + (1 - self.filter_alpha) * self.prev_pos_filtered

        error = -pos_filtered
        error_scaled = error * self.error_scale

        P = self.Kp * error_scaled

        # derivative on measurement (filtered)
        raw_d = (pos_filtered - self.prev_pos_filtered) / max(dt_control, 1e-9)
        self.d_filtered = self.d_filter_alpha * raw_d + (1.0 - self.d_filter_alpha) * self.d_filtered
        # derivative term applied to error; negative sign because derivative of error = -d(pos)/dt
        D = self.Kd * (- self.d_filtered * self.error_scale)

        # conditional integration: only integrate if error small (prevent wind-up)
        if abs(error_scaled) < self.integral_threshold * 100.0:
            self.integral += error_scaled * dt_control

        if self.Ki > 0.0:
            max_int = self.output_limit / max(self.Ki, 1e-9)
            self.integral = np.clip(self.integral, -max_int, max_int)
            I = self.Ki * self.integral
        else:
            I = 0.0

        out = P + I + D
        out = np.clip(out, -self.output_limit, self.output_limit)

        self.prev_pos_filtered = pos_filtered
        return out

# -------------------------
# Plant dynamics
# -------------------------
def step_motor(theta, u_deg, dt):
    """First-order motor: dθ/dt = (u - θ)/tau_m, u in degrees applied as target angle."""
    u = u_deg * deg2rad
    dtheta = (u - theta) / tau_m
    theta_next = theta + dtheta * dt
    return theta_next

def ball_dynamics(x, xdot, theta, dt):
    """Simple rolling ball linearized dynamics (small-angle approx)."""
    accel = (g / (1 + alpha)) * theta - (b / m) * xdot
    xdot_next = xdot + accel * dt
    x_next = x + xdot_next * dt
    return x_next, xdot_next

# -------------------------
# Simulation runner
# -------------------------
def run_sim(controller, label, seed=0):
    np.random.seed(seed)
    controller.reset()
    n_steps = len(t)
    next_control_time = 0.0
    delay_steps = int(round(control_delay / dt))
    meas_buffer = [x0] * (delay_steps + 1)

    x = x0
    xdot = 0.0
    theta = 0.0
    control_out = 0.0

    time_history = []
    x_history = []
    xdot_history = []
    theta_history = []
    control_history = []
    meas_history = []

    for i, ti in enumerate(t):
        # Plant update at high rate
        theta = step_motor(theta, control_out, dt)
        x, xdot = ball_dynamics(x, xdot, theta, dt)

        # delay buffer
        meas_buffer.append(x)
        delayed_meas = meas_buffer.pop(0)

        # Controller update at control rate
        if ti >= next_control_time - 1e-12:
            measured_pos = delayed_meas + np.random.randn() * noise_sigma
            control_out = controller.update(measured_pos, control_dt)
            next_control_time += control_dt

        # Log
        time_history.append(ti)
        x_history.append(x)
        xdot_history.append(xdot)
        theta_history.append(theta)
        control_history.append(control_out)
        meas_history.append(delayed_meas)

    return {
        'time': np.array(time_history),
        'x': np.array(x_history),
        'xdot': np.array(xdot_history),
        'theta': np.array(theta_history),
        'control': np.array(control_history),
        'measured': np.array(meas_history),
        'label': label
    }

# -------------------------
# Main: run baseline vs improved
# -------------------------
def main():
    out_dir = "simulation_and_plotting/sim_outputs"
    os.makedirs(out_dir, exist_ok=True)

    baseline = BaselinePID(Kp=1.8, Ki=0.0, Kd=0.7, filter_alpha=0.7)
    improved = ImprovedPID(Kp=2.2, Ki=0.25, Kd=1.3, filter_alpha=0.85, d_filter_alpha=0.18, integral_threshold=0.6)

    res_b = run_sim(baseline, "Baseline", seed=42)
    res_i = run_sim(improved, "Improved", seed=42)

    # Position plot
    plt.figure(figsize=(10,5))
    plt.plot(res_b['time'], res_b['x'], label='Baseline')
    plt.plot(res_i['time'], res_i['x'], label='Improved')
    plt.xlabel('Time (s)')
    plt.ylabel('X position (m)')
    plt.title('Ball X position: Baseline vs Improved PID')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "position_comparison.png"), dpi=200)

    # Control plot
    plt.figure(figsize=(10,4))
    plt.plot(res_b['time'], res_b['control'], label='Baseline control (deg)')
    plt.plot(res_i['time'], res_i['control'], label='Improved control (deg)')
    plt.xlabel('Time (s)')
    plt.ylabel('Control output (deg)')
    plt.title('Control outputs (degrees)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "control_comparison.png"), dpi=200)

    # Phase plot
    plt.figure(figsize=(6,6))
    plt.plot(res_b['x'], res_b['xdot'], label='Baseline')
    plt.plot(res_i['x'], res_i['xdot'], label='Improved')
    plt.xlabel('X (m)')
    plt.ylabel('Xdot (m/s)')
    plt.title('Phase plot (X vs Xdot)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "phase_plot.png"), dpi=200)

    # Zoomed first 2.5s
    plt.figure(figsize=(10,5))
    plt.plot(res_b['time'], res_b['x'], label='Baseline')
    plt.plot(res_i['time'], res_i['x'], label='Improved')
    plt.xlim(0, 2.5)
    plt.ylim(-0.005, 0.021)
    plt.xlabel('Time (s)')
    plt.ylabel('X position (m)')
    plt.title('Zoom: Ball X position (first 2.5 s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "position_zoom.png"), dpi=200)

    # Metrics
    ov_b, minb = compute_overshoot_from_positive_step(res_b['x'])
    ov_i, mini = compute_overshoot_from_positive_step(res_i['x'])
    settle_b_2 = settling_time_percent(res_b['time'], res_b['x'], pct=0.02)
    settle_i_2 = settling_time_percent(res_i['time'], res_i['x'], pct=0.02)
    settle_b_5 = settling_time_percent(res_b['time'], res_b['x'], pct=0.05)
    settle_i_5 = settling_time_percent(res_i['time'], res_i['x'], pct=0.05)

    print("Simulation results (initial = +20 mm):")
    print("Baseline:   min x = {:.5f} m, overshoot = {:.2f} %, settling time (2%) = {}, (5%) = {}"
          .format(minb, ov_b, settle_b_2, settle_b_5))
    print("Improved:   min x = {:.5f} m, overshoot = {:.2f} %, settling time (2%) = {}, (5%) = {}"
          .format(mini, ov_i, settle_i_2, settle_i_5))
    print()
    print("Saved plots to folder:", os.path.abspath(out_dir))

    # Also show a quick interactive display (optional)
    try:
        plt.show()
    except Exception:
        pass

if __name__ == "__main__":
    main()
