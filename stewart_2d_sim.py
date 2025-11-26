import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from collections import deque

# --- Physical constants and defaults ---
g = 9.81           # gravity (m/s^2)
R = 0.15           # platform radius (m)

# Default parameters
DEFAULT_MASS = 0.1         # kg
DEFAULT_FRICTION = 0.2     # viscous coefficient c (N*s/m)
DEFAULT_MOTOR_TAU = 0.1    # motor time constant (s)
DEFAULT_MOTOR_RATE = np.deg2rad(180)  # max tilt rate (rad/s)
DEFAULT_TILT_MAX = np.deg2rad(15)     # max tilt magnitude (rad)
DEFAULT_CAM_RATE = 30.0    # Hz (think of this as a "baud rate" / FPS)
DEFAULT_DELAY = 0.1        # seconds
DEFAULT_NOISE = 0.002      # meters (std dev)
DEFAULT_KP = 1.5
DEFAULT_KI = 0.0
DEFAULT_KD = 0.4

# Simulation time step (physics integration)
DT = 0.002  # seconds (500 Hz physics)


class PID:
    """Simple discrete PID controller."""
    def __init__(self, Kp, Ki, Kd, Ts):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        # Discrete-time PID (forward Euler integration, backward diff derivative)
        self.integral += error * self.Ts
        derivative = (error - self.prev_error) / self.Ts
        self.prev_error = error
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return u


class Stewart2DSim:
    """
    2D ball-on-disk Stewart platform model:
      - ball dynamics: mass + viscous friction on tilted plane
      - actuator: 1st-order lag + rate & angle limits
      - measurement: sampled, noisy, delayed position (camera)
      - control: PID per axis on measured position
    """
    def __init__(self):
        # State variables
        self.x = 0.05   # m
        self.y = 0.05
        self.vx = 0.0
        self.vy = 0.0

        # Platform tilt (roll about y -> x accel, pitch about x -> y accel)
        self.theta_x = 0.0
        self.theta_y = 0.0

        # Motor commanded tilts
        self.theta_cmd_x = 0.0
        self.theta_cmd_y = 0.0

        # Parameters (will be overwritten from sliders each frame)
        self.mass = DEFAULT_MASS
        self.friction = DEFAULT_FRICTION
        self.motor_tau = DEFAULT_MOTOR_TAU
        self.motor_rate_max = DEFAULT_MOTOR_RATE
        self.tilt_max = DEFAULT_TILT_MAX

        # Camera / measurement
        self.camera_rate = DEFAULT_CAM_RATE
        self.camera_dt = 1.0 / self.camera_rate
        self.next_sample_time = 0.0
        self.delay = DEFAULT_DELAY
        self.noise_std = DEFAULT_NOISE

        # PID controllers (per axis), controller runs at camera_dt
        self.pid_x = PID(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, self.camera_dt)
        self.pid_y = PID(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, self.camera_dt)

        # Measurement buffer for delay
        self.sample_buffer = deque()
        self.delay_samples = max(1, int(self.delay / self.camera_dt))

        # Reference: keep ball at center
        self.ref_x = 0.0
        self.ref_y = 0.0

        # Time
        self.t = 0.0

    def reset(self):
        """Reset state to initial condition."""
        self.x = 0.05
        self.y = 0.05
        self.vx = 0.0
        self.vy = 0.0
        self.theta_x = 0.0
        self.theta_y = 0.0
        self.theta_cmd_x = 0.0
        self.theta_cmd_y = 0.0
        self.next_sample_time = 0.0
        self.t = 0.0
        self.pid_x.reset()
        self.pid_y.reset()
        self.sample_buffer.clear()

    def set_params_from_sliders(self, params):
        """Update plant and controller parameters from the slider values."""
        self.mass = params['mass']
        self.friction = params['friction']
        self.motor_rate_max = np.deg2rad(params['motor_rate_deg'])
        self.tilt_max = np.deg2rad(params['tilt_max_deg'])
        self.camera_rate = params['camera_rate']
        self.camera_dt = 1.0 / self.camera_rate
        self.delay = params['delay']
        self.noise_std = params['noise']
        self.motor_tau = params['motor_tau']

        # Update PID gains/time step if camera rate changed
        self.pid_x.Kp = params['Kp']
        self.pid_x.Ki = params['Ki']
        self.pid_x.Kd = params['Kd']
        self.pid_y.Kp = params['Kp']
        self.pid_y.Ki = params['Ki']
        self.pid_y.Kd = params['Kd']

        # Adjust Ts
        self.pid_x.Ts = self.camera_dt
        self.pid_y.Ts = self.camera_dt

        # Update delay buffer size (in samples)
        self.delay_samples = max(1, int(self.delay / self.camera_dt))
        # Ensure buffer has at least one sample to avoid index errors
        if not self.sample_buffer:
            self.sample_buffer.append((self.ref_x, self.ref_y))

    def physics_step(self, dt):
        """Integrate actuator and ball dynamics for one physics time step."""
        # --- Motor dynamics: first-order with rate limiting and saturation ---
        for axis in ['x', 'y']:
            theta = self.theta_x if axis == 'x' else self.theta_y
            theta_cmd = self.theta_cmd_x if axis == 'x' else self.theta_cmd_y

            # Desired rate based on first-order lag
            desired_rate = (theta_cmd - theta) / max(self.motor_tau, 1e-4)
            # Rate limit
            desired_rate = np.clip(desired_rate,
                                   -self.motor_rate_max,
                                   self.motor_rate_max)
            # Integrate tilt
            theta_new = theta + desired_rate * dt
            # Saturate angle
            theta_new = np.clip(theta_new, -self.tilt_max, self.tilt_max)

            if axis == 'x':
                self.theta_x = theta_new
            else:
                self.theta_y = theta_new

        # --- Ball dynamics: m * a = m g sin(theta) - c v ---
        ax = g * np.sin(self.theta_x) - (self.friction / self.mass) * self.vx
        ay = g * np.sin(self.theta_y) - (self.friction / self.mass) * self.vy

        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Constrain ball within platform radius (simple "falls off & sticks" model)
        r = np.hypot(self.x, self.y)
        if r > R:
            # Project back to circle and zero velocity
            self.x *= R / r
            self.y *= R / r
            self.vx = 0.0
            self.vy = 0.0

    def sample_and_control_if_needed(self):
        """
        Sample camera & run controller at camera_dt.
        Called every physics step, but only does work when t >= next_sample_time.
        """
        if self.t < self.next_sample_time:
            return

        # Take measurement with noise
        meas_x = self.x + np.random.normal(0.0, self.noise_std)
        meas_y = self.y + np.random.normal(0.0, self.noise_std)
        self.sample_buffer.append((meas_x, meas_y))

        # Ensure buffer has enough elements for delay
        while len(self.sample_buffer) < self.delay_samples + 1:
            self.sample_buffer.appendleft((meas_x, meas_y))

        # Delayed measurement
        idx = max(0, len(self.sample_buffer) - 1 - self.delay_samples)
        delayed_meas_x, delayed_meas_y = self.sample_buffer[idx]

        # PID control to compute new commanded tilt
        error_x = self.ref_x - delayed_meas_x
        error_y = self.ref_y - delayed_meas_y

        u_x = self.pid_x.update(error_x)
        u_y = self.pid_y.update(error_y)

        # Commanded tilt is directly PID output, saturated
        self.theta_cmd_x = np.clip(u_x, -self.tilt_max, self.tilt_max)
        self.theta_cmd_y = np.clip(u_y, -self.tilt_max, self.tilt_max)

        # Schedule next sample time
        self.next_sample_time += self.camera_dt

    def step(self, dt):
        """One combined control + physics step."""
        self.sample_and_control_if_needed()
        self.physics_step(dt)
        self.t += dt


# ----------- Matplotlib animation setup ------------

sim = Stewart2DSim()

fig = plt.figure(figsize=(10, 8))
ax_platform = fig.add_subplot(111)
plt.subplots_adjust(left=0.05, right=0.65, top=0.95, bottom=0.25)

# Draw platform circle
theta_circle = np.linspace(0, 2 * np.pi, 200)
ax_platform.plot(R * np.cos(theta_circle), R * np.sin(theta_circle))
ball_point, = ax_platform.plot([], [], 'o', markersize=8)
ax_platform.set_aspect('equal', 'box')
ax_platform.set_xlim(-R * 1.1, R * 1.1)
ax_platform.set_ylim(-R * 1.1, R * 1.1)
ax_platform.set_title("2D Stewart Platform Ball Balancing Simulation")
ax_platform.set_xlabel("x (m)")
ax_platform.set_ylabel("y (m)")

# Text for displaying tilt and time
tilt_text = ax_platform.text(0.02, 0.95,
                             "",
                             transform=ax_platform.transAxes,
                             verticalalignment='top')

# --- Create sliders on the right ---
slider_ax_height = 0.03
slider_ax_spacing = 0.005
start_y = 0.9

slider_defs = [
    ('Kp',           0.0, 10.0, DEFAULT_KP),
    ('Ki',           0.0,  5.0, DEFAULT_KI),
    ('Kd',           0.0,  5.0, DEFAULT_KD),
    ('mass',         0.05, 0.5, DEFAULT_MASS),
    ('friction',     0.0,  1.0, DEFAULT_FRICTION),
    ('motor_rate_deg', 30.0, 360.0, np.rad2deg(DEFAULT_MOTOR_RATE)),
    ('motor_tau',    0.02, 0.5, DEFAULT_MOTOR_TAU),
    ('tilt_max_deg', 5.0, 25.0, np.rad2deg(DEFAULT_TILT_MAX)),
    ('camera_rate',  5.0,  60.0, DEFAULT_CAM_RATE),
    ('delay',        0.0,  0.5, DEFAULT_DELAY),
    ('noise',        0.0,  0.02, DEFAULT_NOISE),
]

sliders = {}

for i, (name, vmin, vmax, vinit) in enumerate(slider_defs):
    y = start_y - i * (slider_ax_height + slider_ax_spacing)
    ax = fig.add_axes([0.7, y, 0.28, slider_ax_height])
    sliders[name] = Slider(ax, name, vmin, vmax, valinit=vinit, valstep=None)

# Reset button
reset_ax = fig.add_axes([0.7, 0.05, 0.1, 0.05])
reset_button = Button(reset_ax, 'Reset')


def get_slider_params():
    return {name: sliders[name].val for name in sliders}


def reset(event):
    sim.reset()
    # Reset sliders to defaults
    for name, vmin, vmax, vinit in slider_defs:
        sliders[name].reset()


reset_button.on_clicked(reset)


def init_anim():
    ball_point.set_data([], [])
    tilt_text.set_text("")
    return ball_point, tilt_text


# Animation update function
def update(frame):
    # Read slider params and update sim's parameters
    params = get_slider_params()
    sim.set_params_from_sliders(params)

    # Step simulation multiple times per frame for smoother motion
    steps_per_frame = int(0.02 / DT)  # ~50 FPS visual
    for _ in range(steps_per_frame):
        sim.step(DT)

    # Update ball position
    ball_point.set_data(sim.x, sim.y)

    # Update text with tilt information (degrees)
    tilt_x_deg = np.rad2deg(sim.theta_x)
    tilt_y_deg = np.rad2deg(sim.theta_y)
    tilt_text.set_text(
        f"tilt_x = {tilt_x_deg:+.1f} deg\n"
        f"tilt_y = {tilt_y_deg:+.1f} deg\n"
        f"time = {sim.t:.2f} s"
    )

    return ball_point, tilt_text


ani = FuncAnimation(fig, update, init_func=init_anim,
                    interval=20, blit=True)

plt.show()
