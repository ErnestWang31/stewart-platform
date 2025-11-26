# Simulated Trajectory Update Experiment Runner
# Regenerates trajectory every Δt based on current position, then follows it with PID
# Uses physics simulation instead of real hardware

import sys
import os
import numpy as np
import json
import time
import csv
from datetime import datetime

# Add parent directory to path to import existing modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pid_controller_2d import PIDController2D
from trajectory_updater import TrajectoryUpdater
from metrics import compute_all_metrics, print_metrics
from physics_engine import StewartPlatformPhysics

class SimTrajectoryUpdateExperiment:
    """Runs a trajectory update experiment with metrics tracking using simulation."""
    
    def __init__(self, config_file="config_stewart.json", sim_config_file="sim_config.json"):
        """Initialize experiment controller."""
        # Load main config
        try:
            with open(config_file, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            print(f"[ERROR] Config file {config_file} not found")
            return
        
        # Load simulation config
        try:
            with open(sim_config_file, 'r') as f:
                sim_config = json.load(f)
                self.config['simulation'] = sim_config.get('simulation', {})
        except FileNotFoundError:
            print(f"[WARNING] Sim config file {sim_config_file} not found, using defaults")
        
        # Initialize physics engine
        self.physics = StewartPlatformPhysics(self.config)
        
        # Initialize PID controller (X-axis only for 1D)
        Kp_x = self.config.get('pid', {}).get('Kp_x', 10.0)
        Ki_x = self.config.get('pid', {}).get('Ki_x', 0.0)
        Kd_x = self.config.get('pid', {}).get('Kd_x', 0.0)
        
        # Max platform tilt is 30° (with 3 motors, one at +15° and two at -15° gives net 30°)
        max_tilt = 30.0
        self.pid = PIDController2D(
            Kp_x=Kp_x, Ki_x=Ki_x, Kd_x=Kd_x,
            Kp_y=0.0, Ki_y=0.0, Kd_y=0.0,  # Y-axis disabled for 1D
            output_limit_x=max_tilt,
            output_limit_y=max_tilt
        )
        
        # Experiment parameters
        self.target_setpoint = 0.0     # Center
        self.update_interval = 0.2     # seconds between trajectory updates (Δt)
        # Trajectory duration mode
        self.use_remaining_time = True  # If True, each trajectory uses remaining time to target
        self.total_trajectory_duration = 3.0  # Total time to reach target (used if use_remaining_time=True)
        self.trajectory_duration = 0.5  # Fixed duration for each segment (used if use_remaining_time=False)
        self.experiment_duration = 10.0  # total experiment duration
        self.tolerance = 0.005  # 5mm completion tolerance
        self.settle_duration = 0.5  # seconds
        self.trajectory_method = 'linear'  # 'linear' or 'polynomial'
        
        # Trajectory updater (will be initialized after ball detection)
        self.trajectory_updater = None
        
        # Data logging
        self.time_log = []
        self.position_x_log = []
        self.setpoint_log = []
        self.error_log = []
        self.control_log = []
        self.tilt_log = []
        self.saturation_log = []
        self.trajectory_update_log = []  # Track when trajectory was updated
        self.trajectory_segments = []  # Store trajectory segments for plotting
        
        # Simulation timing
        self.physics_dt = self.config.get('simulation', {}).get('physics_dt', 0.002)
        self.control_dt = 1.0 / self.config.get('simulation', {}).get('sensor_rate_hz', 30.0)
        
    def run_experiment(self):
        """Run the trajectory update experiment."""
        print("\n" + "="*60)
        print("SIMULATED TRAJECTORY UPDATE EXPERIMENT")
        print("="*60)
        print(f"Trajectory Method: {self.trajectory_method}")
        if self.use_remaining_time:
            print(f"Mode: Remaining Time (Total Duration: {self.total_trajectory_duration} s)")
            print(f"  Each trajectory uses: remaining_time = {self.total_trajectory_duration} - current_time")
        else:
            print(f"Mode: Fixed Duration (Segment Duration: {self.trajectory_duration} s)")
            print(f"  Each trajectory uses: fixed duration = {self.trajectory_duration} s")
        print(f"Update Interval (Δt): {self.update_interval} s")
        print(f"Target Setpoint: {self.target_setpoint*100:.1f} cm (center)")
        print(f"Experiment Duration: {self.experiment_duration} s")
        print(f"Completion Tolerance: {self.tolerance*1000:.1f} mm")
        print("="*60)
        print("Three nested loops:")
        print("  1. Outer: Trajectory recalculation (every Δt)")
        print("  2. Middle: Position update (setpoint from trajectory)")
        print("  3. Inner: PID control loop")
        print("="*60)
        
        # Reset physics to initial state
        self.physics.reset()
        
        print("\n[EXPERIMENT] Starting...")
        
        # Get initial position
        initial_position, _ = self.physics.get_position()
        print(f"[EXPERIMENT] Ball at X={initial_position*100:.1f} cm")
        
        # Initialize trajectory updater
        print(f"\n[EXPERIMENT] Initializing trajectory updater")
        print(f"[EXPERIMENT] Target: {self.target_setpoint*100:.1f} cm")
        print(f"[EXPERIMENT] Will regenerate trajectory every {self.update_interval} s")
        if self.use_remaining_time:
            print(f"[EXPERIMENT] Mode: Remaining time (total duration: {self.total_trajectory_duration} s)")
        else:
            print(f"[EXPERIMENT] Mode: Fixed duration (segment duration: {self.trajectory_duration} s)")
        self.trajectory_updater = TrajectoryUpdater(
            self.target_setpoint,
            self.trajectory_duration,
            self.update_interval,
            method=self.trajectory_method,
            use_remaining_time=self.use_remaining_time,
            total_trajectory_duration=self.total_trajectory_duration if self.use_remaining_time else None
        )
        
        # Reset PID integral terms for clean start
        self.pid.reset_integral()
        
        # Start experiment (t=0 is now)
        experiment_time = 0.0
        next_control_time = 0.0
        
        # Control loop with three nested loops
        print("[EXPERIMENT] Running control loop...")
        control_output_x = 0.0
        control_output_y = 0.0
        
        while experiment_time < self.experiment_duration:
            # Sample sensor and run control at control rate
            if experiment_time >= next_control_time:
                # Get sensor reading (with noise and delay)
                sensor_reading = self.physics.sample_sensor()
                if sensor_reading is None:
                    position_x, position_y = self.physics.get_position()
                    measured_x, measured_y = position_x, position_y
                else:
                    measured_x, measured_y = sensor_reading
                
                # OUTER LOOP: Trajectory recalculation (every Δt)
                trajectory_updated, trajectory_info = self.trajectory_updater.update(measured_x, experiment_time)
                if trajectory_updated and trajectory_info:
                    self.trajectory_update_log.append(experiment_time)
                    # Store trajectory segment for plotting
                    self.trajectory_segments.append(trajectory_info)
                    print(f"[TRAJECTORY] Regenerated at t={experiment_time:.2f}s from x={measured_x*100:.2f}cm")
                
                # MIDDLE LOOP: Position update (get setpoint from current trajectory)
                trajectory_setpoint = self.trajectory_updater.get_setpoint(experiment_time)
                
                # Update PID setpoint
                self.pid.set_setpoint(trajectory_setpoint, 0.0)
                
                # INNER LOOP: PID control (compute control output, pass control_dt for proper timing)
                control_output_x, control_output_y, saturated_x, saturated_y = self.pid.update(measured_x, measured_y, dt=self.control_dt)
                
                # Get current tilt angles
                tilt_x, tilt_y = self.physics.get_tilt_angles()
                
                # Calculate error
                error = trajectory_setpoint - measured_x
                
                # Log data
                self.time_log.append(experiment_time)
                self.position_x_log.append(measured_x)
                self.setpoint_log.append(trajectory_setpoint)
                self.error_log.append(error)
                self.control_log.append(control_output_x)
                self.tilt_log.append(tilt_x)
                self.saturation_log.append(saturated_x)
                
                # Print status every 0.5 seconds
                if len(self.time_log) % int(0.5 / self.control_dt) == 0:
                    traj_info = self.trajectory_updater.get_trajectory_info(experiment_time)
                    update_info = f"next_update={traj_info['time_until_update']:.2f}s"
                    print(f"t={experiment_time:.2f}s | x={measured_x*100:.2f}cm | r={trajectory_setpoint*100:.2f}cm | "
                          f"e={error*100:.2f}cm | u={control_output_x:.2f}° | sat={saturated_x} | [{update_info}]")
                
                next_control_time += self.control_dt
            
            # Advance physics with current control (runs at physics rate)
            self.physics.step(control_output_x, control_output_y, self.physics_dt)
            experiment_time += self.physics_dt
        
        # Stop experiment
        self.physics.step(0, 0, self.physics_dt)
        
        print("\n[EXPERIMENT] Experiment complete!")
        print(f"[EXPERIMENT] Collected {len(self.time_log)} data points")
        print(f"[EXPERIMENT] Trajectory updated {len(self.trajectory_update_log)} times")
        
        # Compute and display metrics
        if len(self.time_log) > 0:
            metrics = compute_all_metrics(
                self.time_log,
                self.position_x_log,
                self.setpoint_log,
                self.error_log,
                self.control_log,
                self.tilt_log,
                self.saturation_log,
                tolerance=self.tolerance,
                settle_duration=self.settle_duration
            )
            
            print_metrics(metrics, "Trajectory Update (Simulated)")
            
            # Save to CSV
            self.save_to_csv()
            
            return metrics
        else:
            print("[ERROR] No data collected")
            return None
    
    def save_to_csv(self):
        """Save experiment data to CSV file."""
        # Create results directory if it doesn't exist
        os.makedirs("results", exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"results/experiment_trajectory_update_{timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'position_x', 'setpoint', 'error', 'control_signal', 'tilt_angle', 'saturated'])
            for i in range(len(self.time_log)):
                writer.writerow([
                    self.time_log[i],
                    self.position_x_log[i],
                    self.setpoint_log[i],
                    self.error_log[i],
                    self.control_log[i],
                    self.tilt_log[i],
                    self.saturation_log[i]
                ])
        
        # Save trajectory segments to JSON for plotting
        segments_filename = f"results/experiment_trajectory_update_{timestamp}_segments.json"
        with open(segments_filename, 'w') as f:
            json.dump(self.trajectory_segments, f, indent=2)
        
        print(f"\n[EXPERIMENT] Data saved to {filename}")
        print(f"[EXPERIMENT] Trajectory segments saved to {segments_filename}")

if __name__ == "__main__":
    # Try to load configs from parent directory
    config_file = "config_stewart.json"
    sim_config_file = "sim_suite/sim_config.json"
    
    if not os.path.exists(config_file):
        config_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config_stewart.json")
    if not os.path.exists(sim_config_file):
        sim_config_file = os.path.join(os.path.dirname(__file__), "sim_config.json")
    
    experiment = SimTrajectoryUpdateExperiment(config_file=config_file, sim_config_file=sim_config_file)
    experiment.run_experiment()
