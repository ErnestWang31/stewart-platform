import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
import os
import tkinter as tk
from tkinter import ttk
from pid_controller_2d import PIDController2D

# --- IMPORT HANDLING ---
GUI_AVAILABLE = False
try:
    import tkinter as tk
    from tkinter import ttk
    import matplotlib
    # CRITICAL: Force Matplotlib to use the Tkinter backend
    matplotlib.use('TkAgg') 
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure # Import Figure explicitly for embedding
    import matplotlib.pyplot as plt
    GUI_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] GUI libraries missing ({e}). Switching to Headless Mode.")
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    GUI_AVAILABLE = False

# --- 1. THE PLANT (Physics Engine) ---
class BallBeamPhysics:
    """Simulates a 1D ball rolling on a tilting beam/plate."""
    def __init__(self, initial_position=-0.10):
        self.x = initial_position  # Position (meters)
        self.v = 0.0               # Velocity (m/s)
        self.g = 9.81              # Gravity
        self.angle = 0.0           # Current plate angle (radians)
        
        # Physics Constant (Hollow Sphere on Plane): a = 0.6 * g * sin(theta)
        self.K_plant = 0.6 * self.g 

    def update(self, command_angle_deg, dt):
        """Advance physics by one time step dt."""
        
        # Physical Constraint: Servo angular limits
        MAX_ANGLE = 15.0 
        command_angle_deg = np.clip(command_angle_deg, -MAX_ANGLE, MAX_ANGLE)
        
        target_angle_rad = np.radians(command_angle_deg)
        
        # Simulate Actuator Dynamics (Servo Lag)
        # Modeled as a Low Pass Filter: Servos move towards target over time
        servo_speed = 0.2 
        self.angle = self.angle * (1 - servo_speed) + target_angle_rad * servo_speed
        
        # Calculate Acceleration
        # Standard convention: Positive angle -> Positive acceleration
        accel = self.K_plant * np.sin(self.angle)
        
        # Euler Integration
        self.v += accel * dt
        self.x += self.v * dt
        
        # Physical Constraint: Ball hits the rim (Inelastic collision)
        if self.x > 0.15:
            self.x = 0.15
            self.v = 0.0
        elif self.x < -0.15:
            self.x = -0.15
            self.v = 0.0

        return self.x

# --- 2. THE SENSOR (Camera + Noise Model) ---
class NoisySensor:
    """Simulates camera noise and transport delay."""
    def __init__(self, noise_std_dev=0.0005, delay_steps=0):
        self.noise_std = noise_std_dev
        self.delay_queue = deque()
        self.delay_steps = delay_steps 

    def read(self, actual_position):
        # Apply Gaussian noise to the true position
        noisy_reading = actual_position + np.random.normal(0, self.noise_std)
        
        # Simulate Processing Latency (Transport Delay)
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
    
    # Initialize Plant (starting at -10cm) and Sensor
    plant = BallBeamPhysics(initial_position=-0.10)
    sensor = NoisySensor(noise_std_dev=0.0005, delay_steps=delay_frames) 
    
    # Initialize Controller
    pid = PIDController2D(Kp_x=Kp, Ki_x=Ki, Kd_x=Kd, output_limit_x=15.0)
    
    # Data Logging
    t_data, x_data, v_data, u_data = [], [], [], []
    
    for i in range(steps):
        t = i * dt
        
        # 1. Sense: Read position from camera model
        measured_x = sensor.read(plant.x)
        
        # 2. Think: Update PID controller
        # Note: Controller internally scales error * 100
        output_x, _ = pid.update(measured_x, 0.0, dt=dt)
        
        # 3. Act: Update Physics model
        plant.update(output_x, dt)
        
        # 4. Log Data
        t_data.append(t)
        x_data.append(plant.x)
        v_data.append(plant.v)
        u_data.append(output_x)
        
    return t_data, x_data, v_data, u_data

# --- 4. MAIN EXECUTION & PLOTTING ---
def main():
    print("Running Stewart Platform Simulation...")
    
    # Scenario 1: Config Defaults (High Stiffness)
    # Gains taken from config_stewart.json
    t1, x1, v1, u1 = run_experiment(
        Kp=0.6, Ki=0.1, Kd=0.4, 
        delay_frames=3, 
        label="Config Defaults"
    )

    # Scenario 2: Tuned for Stability (Critical Damping)
    # Lower P to account for error scaling, Tuned D for damping
    # t2, x2, v2, u2 = run_experiment(
    #     Kp=0.65, Ki=0.0, Kd=0.4, 
    #     delay_frames=3, 
    #     label="Tuned Gains"
    # )
    t2, x2, v2, u2 = run_experiment(
        Kp=2, Ki=0.1, Kd=0.75, 
        delay_frames=3, 
        label="Tuned Gains"
    )

    t3, x3, v3, u3 = run_experiment(
        Kp=1, Ki=0.1, Kd=0.49, 
        delay_frames=3, 
        label="High Latency"
    )

    # Scenario 3: High Latency Stress Test
    # Same tuned gains, but with increased sensor delay
    # t3, x3, v3, u3 = run_experiment(
    #     Kp=0.25, Ki=0.0, Kd=0.45, 
    #     delay_frames=8, 
    #     label="High Latency"
    # )

    # Setup Plots
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
    ax_phase.plot(-0.10, 0, 'ko', label="Start")

    # Plot 3: Control Effort
    ax_ctrl = axes[1, 0]
    ax_ctrl.plot(t1, u1, 'r-', alpha=0.4, label='Baseline')
    ax_ctrl.plot(t2, u2, 'g-', alpha=0.8, label='Tuned')
    ax_ctrl.set_title('Servo Output')
    ax_ctrl.set_ylabel('Angle (deg)')
    ax_ctrl.set_xlabel('Time (s)')
    ax_ctrl.grid(True)

    # Analysis Notes
    ax_txt = axes[1, 1]
    ax_txt.axis('off')
    txt = (
        "Simulation Analysis:\n\n"
        "1. Scaling Factor Effect:\n"
        "   The controller multiplies error by 100.\n"
        "   Config Kp=0.6 acts like Kp=60 on raw position.\n"
        "   High stiffness contributes to initial overshoot.\n\n"
        "2. Sensor Noise:\n"
        "   Without a smoothing filter, sensor noise propagates\n"
        "   to the servo output (see bottom-left plot).\n\n"
        "3. Tuning Results:\n"
        "   Lowering Kp to 0.25 (Green) combined with Kd=0.45\n"
        "   achieves critical damping and eliminates overshoot."
    )
    ax_txt.text(0.05, 0.5, txt, fontsize=11, va='center',
                bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="black"))

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save Plot
    output_dir = "overshoot_plots"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")
    
    save_path = os.path.join(output_dir, "overshoot_simulation_results.png")
    plt.savefig(save_path)
    print(f"Plot saved to: {save_path}")
    
    plt.show()

# --- 5. MODE B: INTERACTIVE GUI ---
def run_interactive_tuner():
    if not GUI_AVAILABLE:
        print("[ERROR] Tkinter not found. Running headless main() instead.")
        main()
        return

    print("Launching Interactive Tuner GUI...")
    
    # 1. Setup Main Window
    root = tk.Tk()
    root.title("PID Simulation Tuner")
    root.geometry("1000x850") # Increased height for text boxes

    # 2. Setup Variables
    kp_var = tk.DoubleVar(value=2.0)
    ki_var = tk.DoubleVar(value=0.1)
    kd_var = tk.DoubleVar(value=0.75)
    delay_var = tk.IntVar(value=3)

    # 3. Setup Matplotlib Figure
    fig = Figure(figsize=(8, 5), dpi=100)
    ax = fig.add_subplot(111)

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)

    def update_simulation(*args):
        # Read sliders
        try:
            kp = float(kp_var.get())
            ki = float(ki_var.get())
            kd = float(kd_var.get())
            delay = int(delay_var.get())
        except ValueError:
            return # Don't update on partial typing

        # Run logic
        t, x, _, _ = run_experiment(kp, ki, kd, delay, "Live")
        
        # Clear and redraw plot using ax object
        ax.clear()
        ax.plot(t, x, 'b-', linewidth=2, label=f'Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}')
        ax.axhline(0, color='k', linestyle=':', alpha=0.5)
        ax.axhspan(-0.02, 0.02, color='yellow', alpha=0.1, label='Integral Window')
        ax.set_ylim(-0.12, 0.05)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.set_title(f'Step Response (Delay: {delay} frames)')
        ax.grid(True)
        ax.legend(loc='upper right')
        
        # Refresh the canvas
        canvas.draw()

    def save_values():
        val_str = f"Kp={kp_var.get():.3f}, Ki={ki_var.get():.3f}, Kd={kd_var.get():.3f}, Delay={delay_var.get()}\n"
        with open("overshoot_plots/good_values.txt", "a") as f:
            f.write(val_str)
        print(f"Values saved: {val_str.strip()}")

    # 4. Create Controls Area
    ctrl_frame = ttk.LabelFrame(root, text="Tuning Parameters")
    ctrl_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=20, pady=20)

    # Slider + Textbox Factory
    def make_input_row(parent, label, var, r_min, r_max, step):
        container = ttk.Frame(parent)
        container.pack(side=tk.TOP, padx=10, pady=5, fill=tk.X)
        
        # Label
        ttk.Label(container, text=f"{label}:", width=15).pack(side=tk.LEFT)
        
        # Text Entry (Direct Value)
        entry = ttk.Entry(container, textvariable=var, width=8)
        entry.pack(side=tk.LEFT, padx=5)
        # Trigger update on Enter key
        entry.bind('<Return>', lambda event: update_simulation())
        
        # Slider
        scale = ttk.Scale(
            container, from_=r_min, to=r_max, variable=var, 
            orient=tk.HORIZONTAL, 
            command=lambda v: update_simulation()
        )
        scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

    # Add Controls
    make_input_row(ctrl_frame, "Kp (Proportional)", kp_var, 0.0, 5.0, 0.01)
    make_input_row(ctrl_frame, "Ki (Integral)", ki_var, 0.0, 2.0, 0.01)
    make_input_row(ctrl_frame, "Kd (Derivative)", kd_var, 0.0, 2.0, 0.01)
    make_input_row(ctrl_frame, "Delay (Frames)", delay_var, 0, 15, 1)

    # Button Bar
    btn_frame = ttk.Frame(root)
    btn_frame.pack(side=tk.BOTTOM, pady=10)
    
    ttk.Button(btn_frame, text="Update Plot (Enter)", command=update_simulation).pack(side=tk.LEFT, padx=10)
    ttk.Button(btn_frame, text="Save Values to File", command=save_values).pack(side=tk.LEFT, padx=10)

    # Initial Draw
    update_simulation()
    
    # Start Loop
    root.mainloop()

if __name__ == "__main__":
    # === SELECT MODE HERE ===
    
    # MODE 1: Run the original comparative report (generates overshoot_plots/ image)
    # main()
    
    # MODE 2: Run the interactive GUI tuner
    run_interactive_tuner()