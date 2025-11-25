# Real Hardware Experiment Runner for Controller Comparison
# Runs three experiments on actual Stewart platform hardware

import numpy as np
import time
import json
import os
import serial
import cv2

from ball_tracker import RealBallTracker
from pid_controller_1d import PIDController1D
from trajectory_generator import generate_trajectory, sample_trajectory
from trajectory_updater import TrajectoryUpdater
from data_logger import DataLogger
from metrics import compute_all_metrics
from inverse_kinematics import StewartPlatformIK

# Experiment parameters
INITIAL_POSITION = 0.10  # 10 cm in meters (will be measured from camera)
TARGET_POSITION = 0.00   # 0 cm in meters
EXPERIMENT_DURATION = 50.0  # seconds
CONTROL_LIMIT = 15.0  # degrees
DT = 0.01  # Control loop time step (100 Hz)

# PID gains (load from config)
DEFAULT_KP = 0.55
DEFAULT_KI = 0.1
DEFAULT_KD = 0.3

# Trajectory parameters
TRAJECTORY_TIME_HORIZON = 4.0  # seconds
TRAJECTORY_UPDATE_PERIOD = 0.2  # seconds

# Metrics parameters
COMPLETION_TOLERANCE = 0.005  # 5 mm
SETTLE_DURATION = 0.5  # seconds


class HardwarePlatformController:
    """Hardware interface for Stewart platform."""
    
    def __init__(self, config_file="config_stewart.json"):
        """Initialize hardware controller.
        
        Args:
            config_file (str): Path to configuration file
        """
        # Load config
        with open(config_file, 'r') as f:
            self.config = json.load(f)
        
        # Servo configuration
        servo_config = self.config.get('servo', {})
        self.servo_port = servo_config.get('port', "COM3")
        self.servo_baud_rate = servo_config.get('baud_rate', 115200)
        self.servo_read_timeout = max(servo_config.get('timeout_seconds', 1.0), 0.0)
        write_timeout_ms = servo_config.get('write_timeout_ms', 50)
        if write_timeout_ms is None:
            write_timeout_ms = 50
        self.servo_write_timeout = max(write_timeout_ms, 0) / 1000.0
        self.neutral_angles = servo_config.get('neutral_angles', [15, 15, 15])
        self.motor_direction_invert = servo_config.get('motor_direction_invert', [False, False, False])
        self.servo_serial = None
        
        # Inverse kinematics
        self.use_inverse_kinematics = self.config.get('platform', {}).get('use_inverse_kinematics', False)
        if self.use_inverse_kinematics:
            self.ik_solver = StewartPlatformIK(self.config)
        else:
            self.ik_solver = None
    
    def connect(self):
        """Connect to Arduino."""
        try:
            self.servo_serial = serial.Serial(
                self.servo_port,
                self.servo_baud_rate,
                timeout=self.servo_read_timeout,
                write_timeout=self.servo_write_timeout
            )
            time.sleep(2)  # Wait for Arduino to initialize
            self.servo_serial.reset_input_buffer()
            print(f"[HARDWARE] Connected to {self.servo_port}")
            return True
        except Exception as e:
            print(f"[HARDWARE] Failed to connect to {self.servo_port}: {e}")
            self.servo_serial = None
            return False
    
    def send_tilt(self, roll_angle, pitch_angle):
        """Send tilt angles to platform (1D: only roll for X-axis).
        
        Args:
            roll_angle (float): Roll angle in degrees (X-axis)
            pitch_angle (float): Pitch angle in degrees (Y-axis, should be 0 for 1D)
        """
        # Clip angles to safe range
        roll_angle = np.clip(roll_angle, -15, 15)
        pitch_angle = np.clip(pitch_angle, -15, 15)
        
        if self.use_inverse_kinematics and self.ik_solver:
            # Use inverse kinematics
            try:
                theta_11, theta_21, theta_31 = self.ik_solver.get_motor_angles(roll_angle, pitch_angle)
                angle_scale = self.config.get('platform', {}).get('ik_angle_scale', 1.0)
                angle_offset = self.config.get('platform', {}).get('ik_angle_offset', 0.0)
                
                motor1_dir = -1.0 if self.motor_direction_invert[0] else 1.0
                motor2_dir = -1.0 if self.motor_direction_invert[1] else 1.0
                motor3_dir = -1.0 if self.motor_direction_invert[2] else 1.0
                
                motor1_angle = self.neutral_angles[0] + (theta_11 * angle_scale + angle_offset) * motor1_dir
                motor2_angle = self.neutral_angles[1] + (theta_21 * angle_scale + angle_offset) * motor2_dir
                motor3_angle = self.neutral_angles[2] + (theta_31 * angle_scale + angle_offset) * motor3_dir
            except Exception as e:
                print(f"[HARDWARE] IK error: {e}, using simplified mapping")
                motor1_angle, motor2_angle, motor3_angle = self._simplified_mapping(roll_angle, pitch_angle)
        else:
            # Use simplified mapping
            motor1_angle, motor2_angle, motor3_angle = self._simplified_mapping(roll_angle, pitch_angle)
        
        # Clip to servo range (0-30 degrees)
        motor1_angle = int(np.clip(motor1_angle, 0, 30))
        motor2_angle = int(np.clip(motor2_angle, 0, 30))
        motor3_angle = int(np.clip(motor3_angle, 0, 30))
        
        # Send to Arduino
        if self.servo_serial:
            try:
                self.servo_serial.write(bytes([motor1_angle, motor2_angle, motor3_angle]))
                self.servo_serial.flush()
            except Exception as e:
                print(f"[HARDWARE] Send error: {e}")
    
    def _simplified_mapping(self, roll_angle, pitch_angle):
        """Simplified trigonometric mapping."""
        motor_angles_deg = self.config.get('motor_angles_deg', None)
        
        if motor_angles_deg and len(motor_angles_deg) == 3:
            motor1_angle_deg = motor_angles_deg[0] - 180
            motor2_angle_deg = motor_angles_deg[1] - 180
            motor3_angle_deg = motor_angles_deg[2] - 180
        else:
            motor1_angle_deg = -90
            motor2_angle_deg = -210
            motor3_angle_deg = -330
        
        motor1_angle_rad = np.radians(motor1_angle_deg)
        motor2_angle_rad = np.radians(motor2_angle_deg)
        motor3_angle_rad = np.radians(motor3_angle_deg)
        
        motor1_height = -roll_angle * np.cos(motor1_angle_rad) - pitch_angle * np.sin(motor1_angle_rad)
        motor2_height = -roll_angle * np.cos(motor2_angle_rad) - pitch_angle * np.sin(motor2_angle_rad)
        motor3_height = -roll_angle * np.cos(motor3_angle_rad) - pitch_angle * np.sin(motor3_angle_rad)
        
        scale_factor = self.config.get('platform', {}).get('motor_scale_factor', 1.0)
        
        motor1_dir = -1.0 if self.motor_direction_invert[0] else 1.0
        motor2_dir = -1.0 if self.motor_direction_invert[1] else 1.0
        motor3_dir = -1.0 if self.motor_direction_invert[2] else 1.0
        
        motor1_angle = self.neutral_angles[0] + motor1_height * scale_factor * motor1_dir
        motor2_angle = self.neutral_angles[1] + motor2_height * scale_factor * motor2_dir
        motor3_angle = self.neutral_angles[2] + motor3_height * scale_factor * motor3_dir
        
        return motor1_angle, motor2_angle, motor3_angle
    
    def reset_to_neutral(self):
        """Reset platform to neutral position."""
        self.send_tilt(0, 0)
    
    def close(self):
        """Close hardware connections."""
        self.reset_to_neutral()
        if self.servo_serial:
            self.servo_serial.close()
            self.servo_serial = None


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


def run_step_pid_experiment_hardware(ball_tracker, platform, config_file="config_stewart.json"):
    """Run Experiment 1: Step PID on real hardware."""
    print("\n" + "="*60)
    print("EXPERIMENT 1: Step PID (HARDWARE)")
    print("="*60)
    
    # Load PID gains
    Kp, Ki, Kd = load_pid_gains_from_config(config_file)
    print(f"PID Gains: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
    
    # Initialize components
    pid = PIDController1D(Kp=Kp, Ki=Ki, Kd=Kd, output_limit=CONTROL_LIMIT)
    logger = DataLogger(experiment_name="experiment1_step_pid_hardware")
    
    # Set setpoint to target
    pid.set_setpoint(TARGET_POSITION)
    
    # Run experiment
    start_time = time.time()
    current_time = 0.0
    
    print(f"Starting experiment: Target = {TARGET_POSITION*100:.1f}cm")
    print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
    print("Press Ctrl+C to stop early\n")
    
    try:
        while current_time < EXPERIMENT_DURATION:
            # Get current ball position from camera
            position = ball_tracker.get_position()
            
            # Update PID controller
            control_signal = pid.update(position, dt=DT)
            
            # Check saturation
            saturated = abs(control_signal) >= CONTROL_LIMIT
            
            # Send control to platform (roll only, pitch = 0 for 1D)
            platform.send_tilt(control_signal, 0.0)
            
            # Calculate error
            error = TARGET_POSITION - position
            
            # Log data
            logger.log(
                time=current_time,
                position_x=position,
                setpoint=TARGET_POSITION,
                error=error,
                control_signal=control_signal,
                tilt_angle=control_signal,
                saturated=saturated
            )
            
            # Update time
            current_time = time.time() - start_time
            
            # Small sleep to maintain timing
            time.sleep(max(0, DT - (time.time() - start_time - current_time)))
    
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Experiment stopped by user")
    
    finally:
        # Reset to neutral
        platform.reset_to_neutral()
    
    logger.save()
    print(f"Experiment complete. Logged {len(logger.data)} data points.")
    return logger


def run_oneshot_trajectory_experiment_hardware(ball_tracker, platform, config_file="config_stewart.json"):
    """Run Experiment 2: One-shot Trajectory on real hardware."""
    print("\n" + "="*60)
    print("EXPERIMENT 2: One-shot Trajectory (HARDWARE)")
    print("="*60)
    
    # Load PID gains
    Kp, Ki, Kd = load_pid_gains_from_config(config_file)
    print(f"PID Gains: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
    
    # Get initial position
    initial_pos = ball_tracker.get_position()
    print(f"Initial position: {initial_pos*100:.2f}cm")
    
    # Initialize components
    pid = PIDController1D(Kp=Kp, Ki=Ki, Kd=Kd, output_limit=CONTROL_LIMIT)
    logger = DataLogger(experiment_name="experiment2_trajectory_oneshot_hardware")
    
    # Generate fixed trajectory
    trajectory = generate_trajectory(
        initial_pos,
        TARGET_POSITION,
        TRAJECTORY_TIME_HORIZON,
        method='polynomial',
        order=5
    )
    print(f"Generated {TRAJECTORY_TIME_HORIZON:.1f}s polynomial trajectory")
    
    # Run experiment
    start_time = time.time()
    current_time = 0.0
    
    print(f"Starting experiment: {initial_pos*100:.1f}cm → {TARGET_POSITION*100:.1f}cm")
    print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
    print("Press Ctrl+C to stop early\n")
    
    try:
        while current_time < EXPERIMENT_DURATION:
            # Get current ball position
            position = ball_tracker.get_position()
            
            # Get setpoint from trajectory
            setpoint = sample_trajectory(trajectory, current_time)
            pid.set_setpoint(setpoint)
            
            # Update PID controller
            control_signal = pid.update(position, dt=DT)
            
            # Check saturation
            saturated = abs(control_signal) >= CONTROL_LIMIT
            
            # Send control to platform
            platform.send_tilt(control_signal, 0.0)
            
            # Calculate error
            error = setpoint - position
            
            # Log data
            logger.log(
                time=current_time,
                position_x=position,
                setpoint=setpoint,
                error=error,
                control_signal=control_signal,
                tilt_angle=control_signal,
                saturated=saturated
            )
            
            # Update time
            current_time = time.time() - start_time
            
            # Small sleep to maintain timing
            time.sleep(max(0, DT - (time.time() - start_time - current_time)))
    
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Experiment stopped by user")
    
    finally:
        # Reset to neutral
        platform.reset_to_neutral()
    
    logger.save()
    print(f"Experiment complete. Logged {len(logger.data)} data points.")
    return logger


def run_trajectory_update_experiment_hardware(ball_tracker, platform, config_file="config_stewart.json"):
    """Run Experiment 3: Trajectory Update on real hardware."""
    print("\n" + "="*60)
    print("EXPERIMENT 3: Trajectory Update (HARDWARE)")
    print("="*60)
    
    # Load PID gains
    Kp, Ki, Kd = load_pid_gains_from_config(config_file)
    print(f"PID Gains: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
    print(f"Update period: {TRAJECTORY_UPDATE_PERIOD:.1f}s")
    
    # Initialize components
    pid = PIDController1D(Kp=Kp, Ki=Ki, Kd=Kd, output_limit=CONTROL_LIMIT)
    trajectory_updater = TrajectoryUpdater(
        target_position=TARGET_POSITION,
        time_horizon=TRAJECTORY_TIME_HORIZON,
        update_period=TRAJECTORY_UPDATE_PERIOD,
        method='polynomial',
        order=5
    )
    logger = DataLogger(experiment_name="experiment3_trajectory_update_hardware")
    
    # Run experiment
    start_time = time.time()
    current_time = 0.0
    
    print(f"Starting experiment: Target = {TARGET_POSITION*100:.1f}cm")
    print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
    print("Press Ctrl+C to stop early\n")
    
    try:
        while current_time < EXPERIMENT_DURATION:
            # Get current ball position
            position = ball_tracker.get_position()
            
            # Update trajectory if needed (for planning, but setpoint is always target)
            trajectory_updater.update(position, current_time)
            
            # Setpoint is always the target (0cm)
            setpoint = TARGET_POSITION
            pid.set_setpoint(setpoint)
            
            # Update PID controller
            control_signal = pid.update(position, dt=DT)
            
            # Check saturation
            saturated = abs(control_signal) >= CONTROL_LIMIT
            
            # Send control to platform
            platform.send_tilt(control_signal, 0.0)
            
            # Calculate error
            error = setpoint - position
            
            # Log data
            logger.log(
                time=current_time,
                position_x=position,
                setpoint=setpoint,
                error=error,
                control_signal=control_signal,
                tilt_angle=control_signal,
                saturated=saturated
            )
            
            # Update time
            current_time = time.time() - start_time
            
            # Small sleep to maintain timing
            time.sleep(max(0, DT - (time.time() - start_time - current_time)))
    
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Experiment stopped by user")
    
    finally:
        # Reset to neutral
        platform.reset_to_neutral()
    
    logger.save()
    print(f"Experiment complete. Logged {len(logger.data)} data points.")
    return logger


def generate_comparison_table(logger1, logger2, logger3):
    """Generate comparison table from three experiment loggers."""
    # Get data arrays
    data1 = logger1.get_data_arrays()
    data2 = logger2.get_data_arrays()
    data3 = logger3.get_data_arrays()
    
    # Compute metrics
    metrics1 = compute_all_metrics(
        data1['position_x'], data1['time'], data1['error'], data1['control_signal'],
        tolerance=COMPLETION_TOLERANCE, settle_duration=SETTLE_DURATION, u_limit=CONTROL_LIMIT
    )
    metrics2 = compute_all_metrics(
        data2['position_x'], data2['time'], data2['error'], data2['control_signal'],
        tolerance=COMPLETION_TOLERANCE, settle_duration=SETTLE_DURATION, u_limit=CONTROL_LIMIT
    )
    metrics3 = compute_all_metrics(
        data3['position_x'], data3['time'], data3['error'], data3['control_signal'],
        tolerance=COMPLETION_TOLERANCE, settle_duration=SETTLE_DURATION, u_limit=CONTROL_LIMIT
    )
    
    # Format functions
    def fmt_time(t):
        return f"{t:.2f}s" if t is not None else "N/A"
    def fmt_float(f):
        return f"{f:.4f}"
    def fmt_percent(p):
        return f"{p:.1f}%"
    
    # Generate table
    table = f"""# Controller Comparison Results (HARDWARE)

## Experiment Parameters
- Target Position: {TARGET_POSITION*100:.1f} cm
- Experiment Duration: {EXPERIMENT_DURATION:.1f} s
- Control Limit: ±{CONTROL_LIMIT:.1f}°
- Completion Tolerance: {COMPLETION_TOLERANCE*1000:.1f} mm
- Settle Duration: {SETTLE_DURATION:.1f} s

## Comparison Table

| Method | Completion Time | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|----------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | {fmt_time(metrics1['completion_time'])} | {fmt_float(metrics1['rmse'])} | {fmt_float(metrics1['max_deviation'])} | {fmt_float(metrics1['max_control'])} | {fmt_percent(metrics1['saturation_percent'])} | {fmt_float(metrics1['max_jerk'])} | {fmt_float(metrics1['rms_jerk'])} | {metrics1['velocity_reversals']} |
| Traj 1-shot | {fmt_time(metrics2['completion_time'])} | {fmt_float(metrics2['rmse'])} | {fmt_float(metrics2['max_deviation'])} | {fmt_float(metrics2['max_control'])} | {fmt_percent(metrics2['saturation_percent'])} | {fmt_float(metrics2['max_jerk'])} | {fmt_float(metrics2['rms_jerk'])} | {metrics2['velocity_reversals']} |
| Traj Update | {fmt_time(metrics3['completion_time'])} | {fmt_float(metrics3['rmse'])} | {fmt_float(metrics3['max_deviation'])} | {fmt_float(metrics3['max_control'])} | {fmt_percent(metrics3['saturation_percent'])} | {fmt_float(metrics3['max_jerk'])} | {fmt_float(metrics3['rms_jerk'])} | {metrics3['velocity_reversals']} |
"""
    
    # Save table
    results_dir = "results"
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    table_file = os.path.join(results_dir, "comparison_table_hardware.md")
    with open(table_file, 'w') as f:
        f.write(table)
    
    print(f"\n[COMPARISON] Comparison table saved to {table_file}")
    print("\n" + "="*60)
    print("COMPARISON TABLE")
    print("="*60)
    print(table)
    
    return metrics1, metrics2, metrics3


def main():
    """Run all three experiments on real hardware."""
    print("\n" + "="*60)
    print("CONTROLLER COMPARISON EXPERIMENTS (HARDWARE)")
    print("="*60)
    print(f"Target Position: {TARGET_POSITION*100:.1f} cm")
    print(f"Experiment Duration: {EXPERIMENT_DURATION:.1f} s")
    print(f"Control Loop Rate: {1/DT:.0f} Hz")
    
    config_file = "config_stewart.json"
    
    # Initialize hardware
    print("\n[INITIALIZING] Setting up hardware...")
    platform = HardwarePlatformController(config_file)
    
    if not platform.connect():
        print("[ERROR] Failed to connect to Arduino. Check connection and port.")
        return
    
    # Initialize camera
    ball_tracker = RealBallTracker(config_file, use_x_axis=True)
    camera_index = platform.config.get('camera', {}).get('index', 0)
    
    print(f"[INITIALIZING] Opening camera {camera_index}...")
    try:
        ball_tracker.initialize_camera(camera_index)
        print("[INITIALIZING] Camera ready")
    except Exception as e:
        print(f"[ERROR] Failed to open camera: {e}")
        platform.close()
        return
    
    # Wait for user to position ball
    print("\n" + "="*60)
    print("SETUP INSTRUCTIONS")
    print("="*60)
    print("1. Position the ball on the platform")
    print("2. Press Enter when ready to start experiments")
    print("3. Each experiment will run for 50 seconds")
    print("4. Press Ctrl+C to stop any experiment early")
    print("="*60)
    input("\nPress Enter to start experiments...")
    
    try:
        # Run experiments
        logger1 = run_step_pid_experiment_hardware(ball_tracker, platform, config_file)
        
        print("\n[PAUSE] Experiment 1 complete. Press Enter to continue to Experiment 2...")
        input()
        
        logger2 = run_oneshot_trajectory_experiment_hardware(ball_tracker, platform, config_file)
        
        print("\n[PAUSE] Experiment 2 complete. Press Enter to continue to Experiment 3...")
        input()
        
        logger3 = run_trajectory_update_experiment_hardware(ball_tracker, platform, config_file)
        
        # Generate comparison table
        metrics1, metrics2, metrics3 = generate_comparison_table(logger1, logger2, logger3)
        
        print("\n[SUCCESS] All experiments completed!")
        print("Run plot_comparison.py to generate visualization plots.")
        print("(Use the hardware CSV files: *_hardware.csv)")
    
    except Exception as e:
        print(f"\n[ERROR] Experiment failed: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\n[CLEANUP] Resetting platform to neutral...")
        platform.reset_to_neutral()
        ball_tracker.close()
        platform.close()
        print("[CLEANUP] Complete")


if __name__ == "__main__":
    main()

