# One-Shot Trajectory Experiment Runner
# Generates a fixed trajectory from initial position to center, then follows it with PID
# Computes and displays metrics

import cv2
import numpy as np
import json
import serial
import time
import csv
from datetime import datetime
from threading import Thread
import queue
from ball_detection_2d import BallDetector2D
from pid_controller_2d import PIDController2D
from inverse_kinematics import StewartPlatformIK
from trajectory_generator import generate_trajectory
from metrics import compute_all_metrics, print_metrics
from disturbance import Disturbance, create_step_disturbance, create_impulse_disturbance, create_sinusoidal_disturbance

class OneShotTrajectoryExperiment:
    """Runs a one-shot trajectory experiment with metrics tracking."""
    
    def __init__(self, config_file="config_stewart.json"):
        """Initialize experiment controller."""
        # Load config
        try:
            with open(config_file, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            print(f"[ERROR] Config file {config_file} not found")
            return
        
        # Initialize PID controller (X-axis only for 1D)
        # Custom PID gains optimized for trajectory tracking with feedforward
        # Lower gains since feedforward handles expected motion, PID mainly corrects errors
        self.trajectory_pid_Kp = 0.6  # Lower than step PID - feedforward does most work
        self.trajectory_pid_Ki = 0.06 # Still need integral for steady-state accuracy
        self.trajectory_pid_Kd = 0.25  # Damping for smooth response
        
        # Allow override from config if specified
        if 'trajectory_pid' in self.config:
            self.trajectory_pid_Kp = self.config['trajectory_pid'].get('Kp_x', self.trajectory_pid_Kp)
            self.trajectory_pid_Ki = self.config['trajectory_pid'].get('Ki_x', self.trajectory_pid_Ki)
            self.trajectory_pid_Kd = self.config['trajectory_pid'].get('Kd_x', self.trajectory_pid_Kd)
        
        self.pid = PIDController2D(
            Kp_x=self.trajectory_pid_Kp, 
            Ki_x=self.trajectory_pid_Ki, 
            Kd_x=self.trajectory_pid_Kd,
            Kp_y=0.0, Ki_y=0.0, Kd_y=0.0,  # Y-axis disabled for 1D
            output_limit_x=self.config.get('platform', {}).get('max_roll_angle', 15.0),
            output_limit_y=15.0
        )
        
        # Set steady-state PID gains (reduced to prevent oscillations)
        self.steady_state_Kp = self.trajectory_pid_Kp * 0.8  # Reduce Kp at steady state
        self.steady_state_Ki = self.trajectory_pid_Ki * 1.0  # Keep Ki for accuracy
        self.steady_state_Kd = self.trajectory_pid_Kd * 0.3  # Significantly reduce Kd (derivative causes oscillations)
        
        print(f"[PID] Using trajectory-specific PID gains (optimized for feedforward):")
        print(f"      Trajectory: Kp={self.trajectory_pid_Kp:.2f}, Ki={self.trajectory_pid_Ki:.2f}, Kd={self.trajectory_pid_Kd:.2f}")
        print(f"      Steady-state: Kp={self.steady_state_Kp:.2f}, Ki={self.steady_state_Ki:.2f}, Kd={self.steady_state_Kd:.2f}")
        
        # Initialize ball detector
        self.detector = BallDetector2D(config_file)
        
        # Initialize inverse kinematics solver
        self.ik_solver = StewartPlatformIK(self.config)
        
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
        
        self.use_inverse_kinematics = self.config.get('platform', {}).get('use_inverse_kinematics', False)
        
        # Experiment parameters
        self.target_setpoint = 0.0     # Center
        self.trajectory_duration = 1.5  # seconds for trajectory
        self.experiment_duration = 15.0  # total experiment duration (extended to evaluate steady-state)
        self.tolerance = 0.005  # 5mm completion tolerance
        self.settle_duration = 0.5  # seconds
        self.trajectory_method = 'min_jerk'  # 'linear', 'polynomial', 'exponential', or 'min_jerk'
        self.trajectory_curvature = 2.0  # Curvature parameter for exponential method (higher = more curved)
        
        # Acceleration feedforward configuration
        self.use_acceleration_feedforward = True  # Enable acceleration-based feedforward for better tracking
        self.K_plant = 0.6 * 9.81  # Physics constant: a = K_plant * sin(theta) for hollow sphere
        self.velocity_feedforward_gain = 0.5  # Gain for velocity feedforward (tune based on system response)
        
        # Steady-state detection thresholds
        self.steady_state_error_threshold = 0.01  # Consider steady state when error < 1cm
        self.steady_state_velocity_threshold = 0.02  # Consider steady state when velocity < 2cm/s
        
        # Disturbance configuration
        self.disturbance_type = 'actuator'  # 'position' or 'actuator' - actuator is more realistic and visible
        # Impulse disturbance at 1.0s (during trajectory following)
        # Actuator impulse disturbance (applied to platform tilt - more realistic, harder to correct):
        self.disturbance = create_impulse_disturbance(time=1.0, magnitude=3.0, duration=1.0, apply_to='position')  # 3° tilt for 1s
        # Alternative disturbances (uncomment to use):
        # Position disturbance (applied to measurement - PID corrects quickly):
        # self.disturbance = create_impulse_disturbance(time=1.0, magnitude=0.1, duration=1.0, apply_to='position')
        # Continuous (sinusoidal) actuator disturbance (oscillating platform tilt):
        # self.disturbance = create_sinusoidal_disturbance(start_time=1.0, amplitude=2.0, frequency=1.0, duration=3.0, apply_to='actuator')  # 2° oscillation at 1Hz for 3s
        
        # Trajectory (will be generated after ball detection)
        self.trajectory = None
        
        # Data logging
        self.time_log = []
        self.position_x_log = []
        self.setpoint_log = []
        self.error_log = []
        self.control_log = []
        self.tilt_log = []
        self.saturation_log = []
        
        # Velocity estimation (for feedforward)
        self.prev_position_x = None
        self.prev_time = None
        self.estimated_velocity_x = 0.0
        self.velocity_filter_alpha = 0.3  # Low-pass filter for velocity estimation
        
        # Thread-safe queue for ball position
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False
        self.start_time = None
        
    def connect_servos(self):
        """Try to open serial connection to Arduino."""
        try:
            self.servo_serial = serial.Serial(
                self.servo_port,
                self.servo_baud_rate,
                timeout=self.servo_read_timeout,
                write_timeout=self.servo_write_timeout
            )
            time.sleep(2)
            self.servo_serial.reset_input_buffer()
            print(f"[ARDUINO] Connected to {self.servo_port}")
            return True
        except Exception as e:
            print(f"[ARDUINO] Failed to connect to {self.servo_port}: {e}")
            self.servo_serial = None
            return False
    
    def send_platform_tilt(self, roll_angle, pitch_angle):
        """Send tilt angles to platform motors."""
        roll_angle = np.clip(roll_angle, -15, 15)
        pitch_angle = np.clip(pitch_angle, -15, 15)
        
        if self.use_inverse_kinematics:
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
                print(f"[IK] Error: {e}, falling back to simplified method")
                motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(roll_angle, pitch_angle)
        else:
            motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(roll_angle, pitch_angle)
        
        motor1_angle = int(np.clip(motor1_angle, 0, 30))
        motor2_angle = int(np.clip(motor2_angle, 0, 30))
        motor3_angle = int(np.clip(motor3_angle, 0, 30))
        
        if self.servo_serial:
            try:
                self.servo_serial.write(bytes([motor1_angle, motor2_angle, motor3_angle]))
                self.servo_serial.flush()
            except Exception as e:
                print(f"[ARDUINO] Send failed: {e}")
    
    def _simplified_motor_mapping(self, roll_angle, pitch_angle):
        """Simplified trigonometric mapping."""
        motor_angles_deg = self.config.get('motor_angles_deg', None)
        
        if motor_angles_deg and len(motor_angles_deg) == 3:
            motor1_angle_deg = motor_angles_deg[0]-180
            motor2_angle_deg = motor_angles_deg[1]-180
            motor3_angle_deg = motor_angles_deg[2]-180
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
    
    def camera_thread(self):
        """Dedicated thread for video capture and ball detection."""
        camera_index = self.config['camera']['index']
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        frame_width = self.config['camera']['frame_width']
        frame_height = self.config['camera']['frame_height']
        
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.resize(frame, (frame_width, frame_height))
            
            # Detect ball position
            found, center, radius, position_x_m, position_y_m = self.detector.detect_ball(frame)
            
            if found:
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait((position_x_m, position_y_m))
                except Exception:
                    pass
            
            # Show processed video
            vis_frame, _, _, _ = self.detector.draw_detection(frame)
            cv2.imshow("One-Shot Trajectory Experiment - Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    def run_experiment(self):
        """Run the one-shot trajectory experiment."""
        print("\n" + "="*60)
        print("ONE-SHOT TRAJECTORY EXPERIMENT")
        print("="*60)
        print(f"Trajectory Method: {self.trajectory_method}")
        print(f"Trajectory Duration: {self.trajectory_duration} s")
        print(f"Target Setpoint: {self.target_setpoint*100:.1f} cm (center)")
        print(f"Experiment Duration: {self.experiment_duration} s")
        print(f"Completion Tolerance: {self.tolerance*1000:.1f} mm")
        print("="*60)
        
        # Connect to servos
        if not self.connect_servos():
            print("[WARNING] No servos connected - running in simulation mode")
        
        # Start camera thread
        self.running = True
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        cam_thread.start()
        
        # Wait a moment for camera to initialize
        time.sleep(1.0)
        
        print("\n[EXPERIMENT] Starting...")
        print("[EXPERIMENT] Waiting for ball detection...")
        
        # Wait for initial ball detection to get starting position
        initial_position = None
        timeout = 5.0
        start_wait = time.time()
        while (time.time() - start_wait) < timeout:
            try:
                position_x, position_y = self.position_queue.get(timeout=0.1)
                initial_position = position_x
                print(f"[EXPERIMENT] Ball detected at X={position_x*100:.1f} cm")
                break
            except queue.Empty:
                continue
        
        if initial_position is None:
            print("[ERROR] Ball not detected. Please ensure ball is visible.")
            self.running = False
            return
        
        # Get fresh ball position and estimate velocity before starting experiment
        print("\n[EXPERIMENT] Getting current ball position and estimating velocity for trajectory generation...")
        position_samples = []
        time_samples = []
        
        # Collect a few position samples to estimate velocity
        sample_start_time = time.time()
        for i in range(5):  # Collect 5 samples
            try:
                position_x, position_y = self.position_queue.get(timeout=0.1)
                position_samples.append(position_x)
                time_samples.append(time.time())
            except queue.Empty:
                break
        
        if len(position_samples) == 0:
            current_position = initial_position  # Fallback
            initial_velocity = 0.0
            print(f"[WARNING] Could not get position samples, using initial: {current_position*100:.1f} cm")
        else:
            current_position = position_samples[-1]  # Use most recent position
            print(f"[EXPERIMENT] Current ball position: {current_position*100:.1f} cm")
            
            # Estimate velocity from samples
            if len(position_samples) >= 2:
                # Use linear regression on recent samples
                dt_total = time_samples[-1] - time_samples[0]
                if dt_total > 0.01:  # Need at least 10ms of data
                    initial_velocity = (position_samples[-1] - position_samples[0]) / dt_total
                    print(f"[EXPERIMENT] Estimated initial velocity: {initial_velocity*100:.2f} cm/s")
                else:
                    initial_velocity = 0.0
            else:
                initial_velocity = 0.0
        
        # Set experiment start time BEFORE generating trajectory (t=0 synchronization)
        experiment_start_time = time.time()
        
        # Generate trajectory from current position to center, accounting for initial velocity
        print(f"\n[EXPERIMENT] Generating trajectory from {current_position*100:.1f} cm to {self.target_setpoint*100:.1f} cm")
        print(f"[EXPERIMENT] Initial velocity: {initial_velocity*100:.2f} cm/s")
        print(f"[EXPERIMENT] Trajectory duration: {self.trajectory_duration} s")
        print(f"[EXPERIMENT] Trajectory method: {self.trajectory_method}")
        if self.trajectory_method == 'exponential':
            print(f"[EXPERIMENT] Curvature parameter: {self.trajectory_curvature}")
        self.trajectory = generate_trajectory(
            current_position,
            self.target_setpoint,
            self.trajectory_duration,
            method=self.trajectory_method,
            curvature=self.trajectory_curvature,
            v0=initial_velocity,
            vf=0.0  # Want to stop at target
        )
        
        print(f"[EXPERIMENT] Starting trajectory following (t=0)")
        
        # Control loop
        print("[EXPERIMENT] Running control loop...")
        while self.running:
            try:
                # Get ball position
                position_x, position_y = self.position_queue.get(timeout=0.1)
                
                # Get current time relative to experiment start
                current_time = time.time() - experiment_start_time
                
                # Estimate ball velocity (for feedforward)
                if self.prev_position_x is not None and self.prev_time is not None:
                    dt_vel = current_time - self.prev_time
                    if dt_vel > 0.001:  # Avoid division by very small numbers
                        # Calculate instantaneous velocity
                        instant_velocity = (position_x - self.prev_position_x) / dt_vel
                        # Low-pass filter to smooth velocity estimate
                        self.estimated_velocity_x = (self.velocity_filter_alpha * instant_velocity + 
                                                     (1 - self.velocity_filter_alpha) * self.estimated_velocity_x)
                self.prev_position_x = position_x
                self.prev_time = current_time
                
                # Apply disturbance if enabled
                if self.disturbance:
                    if self.disturbance_type == 'position':
                        # Apply to position measurement
                        position_x = self.disturbance.apply(position_x, current_time)
                    # Note: actuator disturbance applied after PID computation
                
                # Update setpoint based on trajectory (position update loop)
                trajectory_complete = self.trajectory and self.trajectory.is_complete(current_time)
                trajectory_just_completed = False
                
                if self.trajectory and not trajectory_complete:
                    # Follow trajectory
                    trajectory_setpoint = self.trajectory.get_position(current_time)
                else:
                    # Trajectory complete, hold at target
                    trajectory_setpoint = self.target_setpoint
                    # Check if trajectory just completed (first time we detect completion)
                    if self.trajectory and not hasattr(self, '_trajectory_completed_flag'):
                        trajectory_just_completed = True
                        self._trajectory_completed_flag = True
                
                # Update PID setpoint
                self.pid.set_setpoint(trajectory_setpoint, 0.0)
                
                # Detect steady state: trajectory complete, small error, low velocity
                position_error = abs(trajectory_setpoint - position_x)
                is_steady_state = (trajectory_complete and 
                                  position_error < self.steady_state_error_threshold and
                                  abs(self.estimated_velocity_x) < self.steady_state_velocity_threshold)
                
                # CRITICAL FIX: Reset integral when trajectory completes to prevent overshoot
                # The integral accumulates error during trajectory following, and when feedforward
                # stops, this accumulated error causes overshoot. By resetting the integral,
                # we prevent the controller from "remembering" tracking errors that are no longer relevant.
                if trajectory_just_completed:
                    self.pid.reset_integral_x()
                    print(f"[EXPERIMENT] Trajectory completed at t={current_time:.2f}s - Reset PID integral to prevent overshoot")
                
                # Switch to steady-state PID gains if in steady state
                if is_steady_state:
                    self.pid.set_gains_x(self.steady_state_Kp, self.steady_state_Ki, self.steady_state_Kd)
                else:
                    # Use normal trajectory tracking gains
                    self.pid.set_gains_x(self.trajectory_pid_Kp, self.trajectory_pid_Ki, self.trajectory_pid_Kd)
                
                # Compute control output (PID loop)
                control_output_x, control_output_y, saturated_x, saturated_y = self.pid.update(position_x, position_y)
                
                # Add acceleration and velocity feedforward if enabled
                # IMPORTANT: Only use feedforward during trajectory following, NOT at steady state
                if self.use_acceleration_feedforward and self.trajectory and not self.trajectory.is_complete(current_time) and not is_steady_state:
                    # Get desired acceleration and velocity from trajectory
                    desired_acceleration = self.trajectory.get_acceleration(current_time)
                    desired_velocity = self.trajectory.get_velocity(current_time)
                    
                    # Velocity error: how much faster/slower we need to go
                    velocity_error = desired_velocity - self.estimated_velocity_x
                    
                    # Convert acceleration to tilt angle: a = K_plant * sin(theta)
                    # Therefore: theta = arcsin(a / K_plant)
                    accel_ratio = desired_acceleration / self.K_plant
                    accel_ratio = np.clip(accel_ratio, -1.0, 1.0)  # Clamp for arcsin
                    accel_feedforward_rad = np.arcsin(accel_ratio)
                    accel_feedforward_deg = np.degrees(accel_feedforward_rad)
                    
                    # Velocity feedforward: additional tilt to correct velocity error
                    # Approximate: delta_theta ≈ delta_v / (K_plant * dt) for small changes
                    # More accurate: use proportional correction based on velocity error
                    # For ball dynamics: velocity changes with acceleration, so we need
                    # additional tilt proportional to velocity error
                    velocity_feedforward_deg = self.velocity_feedforward_gain * velocity_error * 100  # Scale from m/s to reasonable tilt
                    velocity_feedforward_deg = np.clip(velocity_feedforward_deg, -5.0, 5.0)  # Limit velocity feedforward
                    
                    # Combine feedforward terms
                    feedforward_tilt_deg = accel_feedforward_deg + velocity_feedforward_deg
                    
                    # Combine feedforward with PID output
                    control_output_x = control_output_x + feedforward_tilt_deg
                
                # Apply actuator disturbance if enabled
                if self.disturbance and self.disturbance_type == 'actuator':
                    control_output_x = self.disturbance.apply_to_actuator(control_output_x, current_time)
                
                # Send control command
                self.send_platform_tilt(control_output_x, control_output_y)
                
                # Calculate error
                error = trajectory_setpoint - position_x
                
                # Log data
                self.time_log.append(current_time)
                self.position_x_log.append(position_x)
                self.setpoint_log.append(trajectory_setpoint)
                self.error_log.append(error)
                self.control_log.append(control_output_x)
                self.tilt_log.append(control_output_x)  # Tilt = control for now
                self.saturation_log.append(saturated_x)
                
                # Check if experiment duration reached
                if current_time >= self.experiment_duration:
                    print(f"[EXPERIMENT] Duration reached ({self.experiment_duration} s)")
                    break
                
                # Print status every 0.5 seconds
                if len(self.time_log) % 15 == 0:  # ~30 Hz, so every 0.5s
                    traj_status = "following" if not self.trajectory.is_complete(current_time) else "complete"
                    print(f"t={current_time:.2f}s | x={position_x*100:.2f}cm | r={trajectory_setpoint*100:.2f}cm | "
                          f"e={error*100:.2f}cm | u={control_output_x:.2f}° | sat={saturated_x} | [{traj_status}]")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[ERROR] Control loop error: {e}")
                break
        
        # Stop experiment
        self.running = False
        self.send_platform_tilt(0, 0)
        if self.servo_serial:
            self.servo_serial.close()
        
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
            
            print_metrics(metrics, "One-Shot Trajectory")
            
            # Save to CSV
            self.save_to_csv()
            
            return metrics
        else:
            print("[ERROR] No data collected")
            return None
    
    def save_to_csv(self):
        """Save experiment data to CSV file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"results/experiment_oneshot_trajectory_{timestamp}.csv"
        
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
    experiment = OneShotTrajectoryExperiment()
    experiment.run_experiment()

