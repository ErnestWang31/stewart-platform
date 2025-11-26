# Simulated Step PID Experiment Runner
# Runs a Step PID experiment using physics simulation instead of real hardware
# Computes and displays metrics

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
from metrics import compute_all_metrics, print_metrics
from physics_engine import StewartPlatformPhysics

class SimStepPIDExperiment:
    """Runs a Step PID experiment with metrics tracking using simulation."""
    
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
                # Merge simulation config into main config
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
        # For Step PID: start at initial position (14cm), then step to center (0cm)
        self.initial_setpoint = self.config.get('simulation', {}).get('initial_position_x', 0.14)  # Use config initial position
        self.target_setpoint = 0.0     # Center
        self.experiment_duration = 10.0  # seconds
        self.tolerance = 0.005  # 5mm completion tolerance
        self.settle_duration = 0.5  # seconds
        
        # Data logging
        self.time_log = []
        self.position_x_log = []
        self.setpoint_log = []
        self.error_log = []
        self.control_log = []
        self.tilt_log = []
        self.saturation_log = []
        
        # Simulation timing
        self.physics_dt = self.config.get('simulation', {}).get('physics_dt', 0.002)
        self.control_dt = 1.0 / self.config.get('simulation', {}).get('sensor_rate_hz', 30.0)
        
    def run_experiment(self):
        """Run the Step PID experiment."""
        print("\n" + "="*60)
        print("SIMULATED STEP PID EXPERIMENT")
        print("="*60)
        print(f"Initial Setpoint: {self.initial_setpoint*100:.1f} cm")
        print(f"Target Setpoint: {self.target_setpoint*100:.1f} cm (center)")
        print(f"Experiment Duration: {self.experiment_duration} s")
        print(f"Completion Tolerance: {self.tolerance*1000:.1f} mm")
        print("="*60)
        
        # Reset physics to initial state (ball starts at initial_position_x from config)
        self.physics.reset()
        
        # Get initial position (should be at initial_position_x from config)
        initial_position, _ = self.physics.get_position()
        print("\n[EXPERIMENT] Starting...")
        print(f"[EXPERIMENT] Ball initial position: {initial_position*100:.1f} cm")
        print(f"[EXPERIMENT] Initial setpoint: {initial_position*100:.1f} cm (same as initial position)")
        print(f"[EXPERIMENT] Target setpoint (at t=0): {self.target_setpoint*100:.1f} cm")
        
        # Set initial setpoint to match initial position (no movement yet)
        self.pid.set_setpoint(initial_position, 0.0)
        current_setpoint = initial_position
        
        # Change setpoint to 0cm immediately (this is t=0 for the experiment)
        print(f"\n[EXPERIMENT] Changing setpoint to {self.target_setpoint*100:.1f} cm (t=0)")
        current_setpoint = self.target_setpoint
        self.pid.set_setpoint(current_setpoint, 0.0)
        
        # Reset PID integral terms for clean start
        self.pid.reset_integral()
        
        # Start experiment timer (t=0 is now)
        experiment_time = 0.0
        next_control_time = 0.0
        
        # Control loop
        print("[EXPERIMENT] Running control loop...")
        control_output_x = 0.0
        control_output_y = 0.0
        
        while experiment_time < self.experiment_duration:
            # Sample sensor and run control at control rate
            if experiment_time >= next_control_time:
                # Get sensor reading (with noise and delay)
                sensor_reading = self.physics.sample_sensor()
                if sensor_reading is None:
                    # Not ready yet, use true position
                    position_x, position_y = self.physics.get_position()
                    measured_x, measured_y = position_x, position_y
                else:
                    measured_x, measured_y = sensor_reading
                
                # Compute control output (pass control_dt for proper timing in simulation)
                control_output_x, control_output_y, saturated_x, saturated_y = self.pid.update(measured_x, measured_y, dt=self.control_dt)
                
                # Get current tilt angles
                tilt_x, tilt_y = self.physics.get_tilt_angles()
                
                # Calculate error
                error = current_setpoint - measured_x
                
                # Log data
                self.time_log.append(experiment_time)
                self.position_x_log.append(measured_x)
                self.setpoint_log.append(current_setpoint)
                self.error_log.append(error)
                self.control_log.append(control_output_x)
                self.tilt_log.append(tilt_x)
                self.saturation_log.append(saturated_x)
                
                # Print status every 0.5 seconds
                if len(self.time_log) % int(0.5 / self.control_dt) == 0:
                    print(f"t={experiment_time:.2f}s | x={measured_x*100:.2f}cm | r={current_setpoint*100:.2f}cm | "
                          f"e={error*100:.2f}cm | u={control_output_x:.2f}° | sat={saturated_x}")
                
                next_control_time += self.control_dt
            
            # Advance physics with current control (runs at physics rate)
            self.physics.step(control_output_x, control_output_y, self.physics_dt)
            experiment_time += self.physics_dt
        
        # Stop experiment
        self.physics.step(0, 0, self.physics_dt)
        
        print("\n[EXPERIMENT] Experiment complete!")
        print(f"[EXPERIMENT] Collected {len(self.time_log)} data points")
        
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
            
            print_metrics(metrics, "Step PID (Simulated)")
            
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
        filename = f"results/experiment_step_pid_{timestamp}.csv"
        
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
        
        print(f"\n[EXPERIMENT] Data saved to {filename}")

if __name__ == "__main__":
    # Try to load configs from parent directory
    config_file = "config_stewart.json"
    sim_config_file = "sim_suite/sim_config.json"
    
    if not os.path.exists(config_file):
        config_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config_stewart.json")
    if not os.path.exists(sim_config_file):
        sim_config_file = os.path.join(os.path.dirname(__file__), "sim_config.json")
    
    experiment = SimStepPIDExperiment(config_file=config_file, sim_config_file=sim_config_file)
    experiment.run_experiment()
