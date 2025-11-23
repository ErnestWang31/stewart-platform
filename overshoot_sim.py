import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import os
from pid_controller_2d import PIDController2D

# --- 1. THE PLANT (Physics Engine) ---
class BallBeamPhysics:
    """Simulates a 1D ball rolling on a tilting beam/plate."""
    def __init__(self, initial_position=-0.10):
        self.x = initial_position  # Position (meters)
        self.v = 0.0               # Velocity (m/s)
        self.g = 9.81              # Gravity
        self.angle = 0.0           # Current plate angle (radians)
        
        # Physics Constant (from derivation)
        # Hollow Sphere on Plane: a = 0.6 * g * sin(theta)
        self.K_plant = 0.6 * self.g 

    def update(self, command_angle_deg, dt):
        """Advance physics by one time step dt."""
        
        # Simulate Servo Limit (Physical Constraint)
        MAX_ANGLE = 15.0 
        command_angle_deg = np.clip(command_angle_deg, -MAX_ANGLE, MAX_ANGLE)
        
        # Convert to radians
        target_angle_rad = np.radians(command_angle_deg)
        
        # === IMPERFECTION 1: SERVO LAG ===
        # Motors don't move instantly. We model this as a Low Pass Filter.
        # A servo_speed of 0.2 means it moves 20% of the way to the target per step.
        servo_speed = 0.2 
        self.angle = self.angle * (1 - servo_speed) + target_angle_rad * servo_speed
        
        # Calculate Acceleration
        # NOTE: Positive Angle (Tilt Right) -> Positive Acceleration (Roll Right)
        # Your controller assumes Positive Position -> Positive Error -> Positive Output
        # If ball is at -0.10 (Left), Error is +0.10. Output is Positive (Tilt Right).
        # Positive Angle -> Positive Accel -> Ball moves Right (towards 0).
        # This sign convention works for negative starting positions.
        accel = self.K_plant * np.sin(self.angle)
        
        # Euler Integration
        self.v += accel * dt
        self.x += self.v * dt
        
        # Hard Constraint: Ball hits the rim (0.15m radius)
        if self.x > 0.15:
            self.x = 0.15
            self.v = 0.0
        elif self.x < -0.15:
            self.x = -0.15
            self.v = 0.0

        return self.x

# --- 2. THE SENSOR (Camera + Noise) ---
class NoisySensor:
    """Simulates camera noise and transport delay."""
    def __init__(self, noise_std_dev=0.0005, delay_steps=0):
        self.noise_std = noise_std_dev
        self.delay_queue = deque()
        self.delay_steps = delay_steps 

    def read(self, actual_position):
        # === IMPERFECTION 2: SENSOR NOISE ===
        # Your new controller has NO filter. This noise hits the D-term directly.
        noisy_reading = actual_position + np.random.normal(0, self.noise_std)
        
        # === IMPERFECTION 3: LATENCY ===
        self.delay_queue.append(noisy_reading)
        if len(self.delay_queue) > self.delay_steps:
            return self.delay_queue.popleft()
        else:
            return self.delay_queue[0]

# --- 3. EXPERIMENT RUNNER ---
def run_experiment(Kp, Ki, Kd, delay_frames, label):
    dt = 0.01 # 100 Hz simulation loop
    total_time = 4.0
    steps = int(total_time / dt)
    
    # Start at -10cm as requested
    plant = BallBeamPhysics(initial_position=-0.10)
    sensor = NoisySensor(noise_std_dev=0.0005, delay_steps=delay_frames) 
    
    # Initialize YOUR Controller
    pid = PIDController2D(Kp_x=Kp, Ki_x=Ki, Kd_x=Kd, output_limit_x=15.0)
    
    # Logs
    t_data, x_data, v_data, u_data = [], [], [], []
    
    for i in range(steps):
        t = i * dt
        
        # 1. SENSE
        measured_x = sensor.read(plant.x)
        
        # 2. THINK
        # Your controller scales error by 100 internally
        output_x, _ = pid.update(measured_x, 0.0, dt=dt)
        
        # 3. ACT
        plant.update(output_x, dt)
        
        # 4. LOG
        t_data.append(t)
        x_data.append(plant.x)
        v_data.append(plant.v)
        u_data.append(output_x)
        
    return t_data, x_data, v_data, u_data

# --- 4. MAIN & PLOTTING ---
if __name__ == "__main__":
    print("Running Simulation with NEW Controller (Starting at -10cm)...")
    
    # === SCENARIO 1: Config Defaults (Overshoot) ===
    # Using gains from your config_stewart.json
    t1, x1, v1, u1 = run_experiment(
        Kp=0.6, Ki=0.1, Kd=0.4, 
        delay_frames=3, 
        label="Config Defaults"
    )

    # === SCENARIO 2: Tuned for Stability ===
    # Reduced P (to handle the *100 scaling) and Tuned D
    t2, x2, v2, u2 = run_experiment(
        Kp=0.25, Ki=0.0, Kd=0.45, 
        delay_frames=3, 
        label="Tuned Gains"
    )

    # === SCENARIO 3: High Latency ===
    # Same tuned gains, but camera is slower (5 frames delay)
    t3, x3, v3, u3 = run_experiment(
        Kp=0.25, Ki=0.0, Kd=0.45, 
        delay_frames=8, 
        label="High Latency"
    )

    # PLOTTING
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Simulation Results: Overshoot & Latency Analysis', fontsize=16)

    # Plot 1: Position vs Time (Step Response)
    ax_pos = axes[0, 0]
    ax_pos.plot(t1, x1, 'r-', label='Baseline (Config Gains)')
    ax_pos.plot(t2, x2, 'g-', linewidth=2, label='Tuned (Lower P, High D)')
    ax_pos.plot(t3, x3, 'b--', label='High Latency')
    ax_pos.axhline(0, color='k', linestyle=':', alpha=0.5)
    ax_pos.set_title('Step Response: Position vs Time')
    ax_pos.set_ylabel('Position (m)')
    ax_pos.set_xlabel('Time (s)')
    ax_pos.grid(True)
    ax_pos.legend()

    # Plot 2: Phase Plane (Velocity vs Position)
    ax_phase = axes[0, 1]
    ax_phase.plot(x1, v1, 'r-', alpha=0.6)
    ax_phase.plot(x2, v2, 'g-', linewidth=2)
    ax_phase.plot(x3, v3, 'b--', alpha=0.6)
    ax_phase.set_title('Phase Plane (Velocity vs Position)')
    ax_phase.set_xlabel('Position (m)')
    ax_phase.set_ylabel('Velocity (m/s)')
    ax_phase.grid(True)
    ax_phase.axhline(0, color='k', linestyle=':')
    ax_phase.axvline(0, color='k', linestyle=':')
    # Mark start point
    ax_phase.plot(-0.10, 0, 'ko', label="Start")

    # Plot 3: Control Effort
    ax_ctrl = axes[1, 0]
    ax_ctrl.plot(t1, u1, 'r-', alpha=0.4, label='Baseline')
    ax_ctrl.plot(t2, u2, 'g-', alpha=0.8, label='Tuned')
    ax_ctrl.set_title('Servo Output (Notice Noise w/o Filter)')
    ax_ctrl.set_ylabel('Angle (deg)')
    ax_ctrl.set_xlabel('Time (s)')
    ax_ctrl.grid(True)

    # Analysis Text
    ax_txt = axes[1, 1]
    ax_txt.axis('off')
    txt = (
        "Observation on New Controller:\n\n"
        "1. Scaling Factor:\n"
        "   The controller multiplies error by 100.\n"
        "   Config Kp=0.6 acts like Kp=60 on raw position.\n"
        "   This high stiffness causes the red overshoot.\n\n"
        "2. Lack of Filter:\n"
        "   Notice the 'fuzz' on the Servo Output graph.\n"
        "   Without alpha-filter, sensor noise hits the motor.\n"
        "   However, this reduces lag, helping stability.\n\n"
        "3. Tuning:\n"
        "   Lowering Kp to 0.25 (Green) fixes the overshoot\n"
        "   better than just increasing Kd."
    )
    ax_txt.text(0.05, 0.5, txt, fontsize=11, va='center',
                bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="black"))

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # === SAVE RESULTS ===
    output_dir = "overshoot_plots"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")
    
    save_path = os.path.join(output_dir, "overshoot_simulation_results.png")
    plt.savefig(save_path)
    print(f"Plot saved to: {save_path}")
    
    plt.show()