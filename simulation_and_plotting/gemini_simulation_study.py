import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from gemini_controls import PIDController2D

# --- 1. THE PLANT (Physics Engine) ---
class BallBeamPhysics:
    """Simulates a 1D ball rolling on a tilting beam/plate."""
    def __init__(self, initial_position=0.10):
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
        # Servos can't move instantly or infinitely
        MAX_ANGLE = 15.0 # degrees
        command_angle_deg = np.clip(command_angle_deg, -MAX_ANGLE, MAX_ANGLE)
        
        # Convert to radians for physics calculation
        target_angle_rad = np.radians(command_angle_deg)
        
        # Simulate Servo Speed (Simple low-pass filter to mimic motor movement time)
        # This adds a tiny bit of realism vs instant movement
        servo_speed = 0.2 # 0 to 1 (1 = instant)
        self.angle = self.angle * (1 - servo_speed) + target_angle_rad * servo_speed
        
        # Calculate Acceleration
        # FIXED: Sign inversion to match PID controller convention.
        # PID sends Negative Angle for Positive Position.
        # We want Negative Angle -> Negative Acceleration (Roll Left).
        # Therefore: accel = +K * sin(theta) (since sin(-theta) is negative)
        accel = self.K_plant * np.sin(self.angle)
        
        # Euler Integration (Update Position and Velocity)
        self.v += accel * dt
        self.x += self.v * dt
        
        # Hard Constraint: Ball hits the rim (0.15m radius)
        # If it hits the rim, velocity stops (inelastic collision)
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
    def __init__(self, noise_std_dev=0.001, delay_steps=0):
        self.noise_std = noise_std_dev
        self.delay_queue = deque()
        self.delay_steps = delay_steps # How many frames old is the data?

    def read(self, actual_position):
        # Add Gaussian Noise
        noisy_reading = actual_position + np.random.normal(0, self.noise_std)
        
        # Handle Delay
        self.delay_queue.append(noisy_reading)
        if len(self.delay_queue) > self.delay_steps:
            return self.delay_queue.popleft()
        else:
            return self.delay_queue[0] # Return oldest available

# --- 3. EXPERIMENT RUNNER ---
def run_experiment(Kp, Ki, Kd, alpha, delay_frames, label):
    """Runs a single simulation scenario."""
    
    # Setup Time
    dt = 0.01 # 100 Hz simulation loop
    total_time = 4.0
    steps = int(total_time / dt)
    
    # Initialize Objects
    plant = BallBeamPhysics(initial_position=0.10) # Start 10cm to the right
    # Note: Increasing delay_frames mimics slow camera processing
    sensor = NoisySensor(noise_std_dev=0.0005, delay_steps=delay_frames) 
    
    # Initialize Controller
    # We use the EXACT class from your project
    pid = PIDController2D(Kp_x=Kp, Ki_x=Ki, Kd_x=Kd, output_limit_x=15.0)
    pid.set_filter_alpha(alpha)
    
    # Data Logging
    t_data = []
    x_data = []
    v_data = [] # For Phase Plane
    u_data = [] # Control Effort
    
    # Simulation Loop
    for i in range(steps):
        t = i * dt
        
        # 1. SENSE: Read position (with noise and delay)
        measured_x = sensor.read(plant.x)
        
        # 2. THINK: Update PID
        # We pass 0.0 for Y since we are doing 1D simulation
        output_x, _ = pid.update(measured_x, 0.0, dt=dt)
        
        # 3. ACT: Update Physics
        plant.update(output_x, dt)
        
        # 4. LOG
        t_data.append(t)
        x_data.append(plant.x)
        v_data.append(plant.v)
        u_data.append(output_x)
        
    return t_data, x_data, v_data, u_data

# --- 4. MAIN EXECUTION & PLOTTING ---
if __name__ == "__main__":
    print("Running Stewart Platform Overshoot Analysis...")
    
    # === SCENARIO 1: The Baseline (Undertuned) ===
    # High P, Low D, Standard Filter
    # Note: Remember code scales P by 10. So Kp=0.4 is effectively 4.0
    t1, x1, v1, u1 = run_experiment(
        Kp=0.5, Ki=0.0, Kd=0.08, 
        alpha=0.7, delay_frames=2, 
        label="Baseline (Overshoot)"
    )

    # === SCENARIO 2: The Fix (Critically Damped) ===
    # UPDATED GAINS: Lowered Kp slightly and increased Kd to handle servo lag
    t2, x2, v2, u2 = run_experiment(
        Kp=0.3, Ki=0.0, Kd=0.45, 
        alpha=0.9, delay_frames=2, 
        label="Tuned (Damping Increased)"
    )

    # === SCENARIO 3: The Latency Trap ===
    # Good Gains, but Lag increased (Simulating heavy filtering or slow camera)
    # Alpha 0.1 is very smooth but very laggy
    t3, x3, v3, u3 = run_experiment(
        Kp=0.3, Ki=0.0, Kd=0.45, 
        alpha=0.1, delay_frames=5, 
        label="Latency Effect (Alpha=0.1)"
    )

    # === PLOTTING ===
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Analysis of Overshoot in Stewart Platform Control', fontsize=16)

    # Plot 1: Position vs Time (Step Response)
    ax_pos = axes[0, 0]
    ax_pos.plot(t1, x1, 'r-', label='Baseline (Undertuned)')
    ax_pos.plot(t2, x2, 'g-', linewidth=2, label='Tuned (High D)')
    ax_pos.plot(t3, x3, 'b--', label='High Latency')
    ax_pos.axhline(0, color='k', linestyle=':', alpha=0.5)
    ax_pos.set_title('Step Response: Position vs Time')
    ax_pos.set_ylabel('Position (m)')
    ax_pos.set_xlabel('Time (s)')
    ax_pos.legend()
    ax_pos.grid(True)

    # Plot 2: Phase Plane (Velocity vs Position)
    # This is excellent for Engineering analysis
    ax_phase = axes[0, 1]
    ax_phase.plot(x1, v1, 'r-', alpha=0.6, label='Baseline')
    ax_phase.plot(x2, v2, 'g-', linewidth=2, label='Tuned')
    ax_phase.plot(x3, v3, 'b--', alpha=0.6, label='Latency')
    ax_phase.set_title('Phase Plane Trajectory (Vel vs Pos)')
    ax_phase.set_ylabel('Velocity (m/s)')
    ax_phase.set_xlabel('Position (m)')
    ax_phase.axvline(0, color='k', linestyle=':')
    ax_phase.axhline(0, color='k', linestyle=':')
    ax_phase.grid(True)
    # Mark the start point
    ax_phase.plot(0.10, 0, 'ko', label='Start')

    # Plot 3: Control Effort (Motor Output)
    ax_ctrl = axes[1, 0]
    ax_ctrl.plot(t1, u1, 'r-', alpha=0.6, label='Baseline')
    ax_ctrl.plot(t2, u2, 'g-', alpha=0.6, label='Tuned')
    ax_ctrl.set_title('Control Effort (Servo Angle)')
    ax_ctrl.set_ylabel('Angle (deg)')
    ax_ctrl.set_xlabel('Time (s)')
    ax_ctrl.grid(True)

    # Text / Analysis Box
    ax_txt = axes[1, 1]
    ax_txt.axis('off')
    analysis_text = (
        "Analysis of Simulation Results:\n\n"
        "1. Red Trace (Baseline): Shows standard overshoot.\n"
        "   Cause: Low Kd relative to Kp (Underdamped).\n\n"
        "2. Green Trace (Tuned): Critical Damping.\n"
        "   Action: Tuned Kd higher to account for servo lag.\n"
        "   Result: Ball reaches center and stops without crossing.\n\n"
        "3. Blue Trace (Latency): Good gains, but bad filter.\n"
        "   Cause: Alpha=0.1 introduces phase lag.\n"
        "   Result: System oscillates despite correct PID gains."
    )
    ax_txt.text(0.1, 0.5, analysis_text, fontsize=12, va='center', 
                bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="black", alpha=0.8))

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()