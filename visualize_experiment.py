# Visualize Experiment Module
# Shows real-time visualization of a single experiment run

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from ball_tracker import SimulatedBallTracker
from pid_controller_1d import PIDController1D
from trajectory_generator import generate_trajectory, sample_trajectory
from trajectory_updater import TrajectoryUpdater
import json

# Experiment parameters
INITIAL_POSITION = 0.10  # 10 cm in meters
TARGET_POSITION = 0.00   # 0 cm in meters
EXPERIMENT_DURATION = 50.0  # seconds (increased for better visualization)
CONTROL_LIMIT = 15.0  # degrees
DT = 0.01  # Control loop time step (100 Hz)

# PID gains
DEFAULT_KP = 0.55
DEFAULT_KI = 0.1
DEFAULT_KD = 0.3

# Trajectory parameters
TRAJECTORY_TIME_HORIZON = 4.0  # seconds
TRAJECTORY_UPDATE_PERIOD = 0.2  # seconds


def load_pid_gains_from_config(config_file="config_stewart.json"):
    """Load PID gains from config file."""
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
            pid_config = config.get('pid', {})
            Kp = pid_config.get('Kp_x', DEFAULT_KP)
            Ki = pid_config.get('Ki_x', DEFAULT_KI)
            Kd = pid_config.get('Kd_x', DEFAULT_KD)
            return Kp, Ki, Kd
    except Exception as e:
        print(f"[CONFIG] Could not load config: {e}, using defaults")
        return DEFAULT_KP, DEFAULT_KI, DEFAULT_KD


class ExperimentVisualizer:
    """Real-time visualization of experiment."""
    
    def __init__(self, experiment_type='step_pid'):
        """Initialize visualizer.
        
        Args:
            experiment_type (str): 'step_pid', 'oneshot', or 'update'
        """
        self.experiment_type = experiment_type
        self.Kp, self.Ki, self.Kd = load_pid_gains_from_config()
        
        # Initialize components
        self.ball_tracker = SimulatedBallTracker(initial_position=INITIAL_POSITION)
        self.pid = PIDController1D(Kp=self.Kp, Ki=self.Ki, Kd=self.Kd, output_limit=CONTROL_LIMIT)
        
        if experiment_type == 'oneshot':
            self.trajectory = generate_trajectory(
                INITIAL_POSITION,
                TARGET_POSITION,
                TRAJECTORY_TIME_HORIZON,
                method='polynomial',
                order=5
            )
        elif experiment_type == 'update':
            self.trajectory_updater = TrajectoryUpdater(
                target_position=TARGET_POSITION,
                time_horizon=TRAJECTORY_TIME_HORIZON,
                update_period=TRAJECTORY_UPDATE_PERIOD,
                method='polynomial',
                order=5
            )
        
        # Data storage
        self.time_data = []
        self.position_data = []
        self.setpoint_data = []
        self.error_data = []
        self.control_data = []
        self.tilt_data = []
        
        # Time tracking
        self.start_time = None
        self.current_time = 0.0
        self.running = False
        
        # Setup plots
        self.setup_plots()
    
    def setup_plots(self):
        """Setup matplotlib figure and axes."""
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle(f'Experiment Visualization: {self.experiment_type.upper()}', 
                         fontsize=16, fontweight='bold')
        
        # Create subplots
        gs = self.fig.add_gridspec(3, 2, height_ratios=[1, 1, 1], hspace=0.3, wspace=0.3)
        
        # Plot 1: Position and Setpoint
        self.ax1 = self.fig.add_subplot(gs[0, :])
        self.ax1.set_xlabel('Time (s)', fontsize=10)
        self.ax1.set_ylabel('Position (m)', fontsize=10)
        self.ax1.set_title('Position Tracking: x(t) and r(t)', fontsize=12, fontweight='bold')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_xlim(0, EXPERIMENT_DURATION)
        self.ax1.set_ylim(-0.02, 0.12)
        self.line1_pos, = self.ax1.plot([], [], 'b-', linewidth=2, label='Position x(t)')
        self.line1_set, = self.ax1.plot([], [], 'r--', linewidth=2, label='Setpoint r(t)')
        self.ax1.legend(loc='upper right')
        
        # Plot 2: Error
        self.ax2 = self.fig.add_subplot(gs[1, 0])
        self.ax2.set_xlabel('Time (s)', fontsize=10)
        self.ax2.set_ylabel('Error (m)', fontsize=10)
        self.ax2.set_title('Tracking Error: e(t)', fontsize=12, fontweight='bold')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_xlim(0, EXPERIMENT_DURATION)
        self.ax2.axhline(y=0, color='black', linestyle=':', linewidth=1, alpha=0.5)
        self.line2, = self.ax2.plot([], [], 'g-', linewidth=2)
        
        # Plot 3: Control Signal
        self.ax3 = self.fig.add_subplot(gs[1, 1])
        self.ax3.set_xlabel('Time (s)', fontsize=10)
        self.ax3.set_ylabel('Control (degrees)', fontsize=10)
        self.ax3.set_title('Control Signal: u(t)', fontsize=12, fontweight='bold')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_xlim(0, EXPERIMENT_DURATION)
        self.ax3.set_ylim(-16, 16)
        self.ax3.axhline(y=CONTROL_LIMIT, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Limit')
        self.ax3.axhline(y=-CONTROL_LIMIT, color='red', linestyle='--', linewidth=1, alpha=0.5)
        self.line3, = self.ax3.plot([], [], 'orange', linewidth=2)
        self.ax3.legend(loc='upper right')
        
        # Plot 4: Platform visualization
        self.ax4 = self.fig.add_subplot(gs[2, :])
        self.ax4.set_xlabel('Position (m)', fontsize=10)
        self.ax4.set_ylabel('', fontsize=10)
        self.ax4.set_title('Platform View (Ball Position)', fontsize=12, fontweight='bold')
        self.ax4.set_xlim(-0.15, 0.15)
        self.ax4.set_ylim(-0.05, 0.05)
        self.ax4.set_aspect('equal')
        self.ax4.grid(True, alpha=0.3)
        
        # Draw platform
        platform_radius = 0.15
        circle = plt.Circle((0, 0), platform_radius, fill=False, color='gray', linewidth=2)
        self.ax4.add_patch(circle)
        self.ax4.axvline(x=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
        self.ax4.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
        
        # Ball marker
        self.ball_marker, = self.ax4.plot([], [], 'ro', markersize=15, label='Ball')
        self.setpoint_marker, = self.ax4.plot([], [], 'gx', markersize=20, markeredgewidth=3, label='Setpoint')
        self.ax4.legend(loc='upper right')
        
        # Text display
        self.text_display = self.ax4.text(0.02, 0.02, '', fontsize=10, 
                                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                                         transform=self.ax4.transAxes)
    
    def update_setpoint(self, current_time):
        """Update setpoint based on experiment type."""
        if self.experiment_type == 'step_pid':
            return TARGET_POSITION
        elif self.experiment_type == 'oneshot':
            return sample_trajectory(self.trajectory, current_time)
        elif self.experiment_type == 'update':
            # Update trajectory if needed (for planning, but setpoint is always target)
            position = self.ball_tracker.get_position()
            self.trajectory_updater.update(position, current_time)
            # Setpoint is always the target (0cm), trajectory is just for planning
            return TARGET_POSITION
        return TARGET_POSITION
    
    def step(self):
        """Perform one simulation step."""
        if not self.running:
            return
        
        # Get current time
        if self.start_time is None:
            self.start_time = time.time()
            self.current_time = 0.0
        else:
            self.current_time = time.time() - self.start_time
        
        if self.current_time >= EXPERIMENT_DURATION:
            self.running = False
            return
        
        # Get current ball position
        position = self.ball_tracker.get_position()
        
        # Update setpoint
        setpoint = self.update_setpoint(self.current_time)
        self.pid.set_setpoint(setpoint)
        
        # Update PID controller
        control_signal = self.pid.update(position, dt=DT)
        
        # Update ball dynamics
        tilt_angle = control_signal
        self.ball_tracker.update(tilt_angle, dt=DT)
        
        # Store data
        self.time_data.append(self.current_time)
        self.position_data.append(position)
        self.setpoint_data.append(setpoint)
        error = setpoint - position
        self.error_data.append(error)
        self.control_data.append(control_signal)
        self.tilt_data.append(tilt_angle)
        
        # Update plots
        self.update_plots()
    
    def update_plots(self):
        """Update all plot data."""
        if len(self.time_data) == 0:
            return
        
        # Update position plot
        self.line1_pos.set_data(self.time_data, self.position_data)
        self.line1_set.set_data(self.time_data, self.setpoint_data)
        
        # Update error plot
        self.line2.set_data(self.time_data, self.error_data)
        
        # Update control plot
        self.line3.set_data(self.time_data, self.control_data)
        
        # Update platform view
        if len(self.position_data) > 0:
            current_pos = self.position_data[-1]
            current_set = self.setpoint_data[-1]
            self.ball_marker.set_data([current_pos], [0])
            self.setpoint_marker.set_data([current_set], [0])
            
            # Update text
            error = self.error_data[-1] if len(self.error_data) > 0 else 0
            control = self.control_data[-1] if len(self.control_data) > 0 else 0
            text = f'Time: {self.current_time:.2f}s\n'
            text += f'Position: {current_pos*100:.2f}cm\n'
            text += f'Setpoint: {current_set*100:.2f}cm\n'
            text += f'Error: {error*100:.2f}cm\n'
            text += f'Control: {control:.1f}°'
            self.text_display.set_text(text)
        
        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def run(self):
        """Run the experiment with visualization."""
        print(f"\n{'='*60}")
        print(f"Running {self.experiment_type.upper()} Experiment")
        print(f"{'='*60}")
        print(f"Initial Position: {INITIAL_POSITION*100:.1f}cm")
        print(f"Target Position: {TARGET_POSITION*100:.1f}cm")
        print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
        print(f"PID Gains: Kp={self.Kp:.2f}, Ki={self.Ki:.2f}, Kd={self.Kd:.2f}")
        print("\nClose the plot window to stop the experiment.\n")
        
        # Enable interactive mode
        plt.ion()
        self.fig.show()
        
        self.running = True
        self.start_time = time.time()
        
        if self.experiment_type == 'update':
            position = self.ball_tracker.get_position()
            self.trajectory_updater.update(position, 0.0)
        
        # Main simulation loop
        try:
            while self.running and self.current_time < EXPERIMENT_DURATION:
                self.step()
                plt.pause(DT)  # Small pause for real-time effect
                
                # Check if window is closed
                if not plt.get_fignums():
                    self.running = False
                    break
        except KeyboardInterrupt:
            self.running = False
        
        plt.ioff()
        print("\nExperiment complete!")


def main():
    """Main function to run visualization."""
    import sys
    
    # Get experiment type from command line or default
    if len(sys.argv) > 1:
        exp_type = sys.argv[1].lower()
        if exp_type not in ['step_pid', 'oneshot', 'update']:
            print("Invalid experiment type. Use: step_pid, oneshot, or update")
            return
    else:
        # Interactive selection
        print("\nSelect experiment type:")
        print("1. Step PID (setpoint jumps from 10cm → 0cm)")
        print("2. One-shot Trajectory (fixed trajectory)")
        print("3. Trajectory Update (replan every 0.2s)")
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == '1':
            exp_type = 'step_pid'
        elif choice == '2':
            exp_type = 'oneshot'
        elif choice == '3':
            exp_type = 'update'
        else:
            print("Invalid choice, defaulting to step_pid")
            exp_type = 'step_pid'
    
    # Create and run visualizer
    visualizer = ExperimentVisualizer(experiment_type=exp_type)
    visualizer.run()


if __name__ == "__main__":
    main()

