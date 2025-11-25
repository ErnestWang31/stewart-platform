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
        Kp_x = self.config.get('pid', {}).get('Kp_x', 10.0)
        Ki_x = self.config.get('pid', {}).get('Ki_x', 0.0)
        Kd_x = self.config.get('pid', {}).get('Kd_x', 0.0)
        
        self.pid = PIDController2D(
            Kp_x=Kp_x, Ki_x=Ki_x, Kd_x=Kd_x,
            Kp_y=0.0, Ki_y=0.0, Kd_y=0.0,  # Y-axis disabled for 1D
            output_limit_x=self.config.get('platform', {}).get('max_roll_angle', 15.0),
            output_limit_y=15.0
        )
        
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
        self.trajectory_duration = 5.0  # seconds for trajectory
        self.experiment_duration = 10.0  # total experiment duration (extended to evaluate steady-state)
        self.tolerance = 0.005  # 5mm completion tolerance
        self.settle_duration = 0.5  # seconds
        self.trajectory_method = 'linear'  # 'linear' or 'polynomial'
        
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
        
        # Generate trajectory from initial position to center
        print(f"\n[EXPERIMENT] Generating trajectory from {initial_position*100:.1f} cm to {self.target_setpoint*100:.1f} cm")
        print(f"[EXPERIMENT] Trajectory duration: {self.trajectory_duration} s")
        self.trajectory = generate_trajectory(
            initial_position,
            self.target_setpoint,
            self.trajectory_duration,
            method=self.trajectory_method
        )
        
        # Start experiment (t=0 is now)
        experiment_start_time = time.time()
        print(f"[EXPERIMENT] Starting trajectory following (t=0)")
        
        # Control loop
        print("[EXPERIMENT] Running control loop...")
        while self.running:
            try:
                # Get ball position
                position_x, position_y = self.position_queue.get(timeout=0.1)
                
                # Get current time relative to experiment start
                current_time = time.time() - experiment_start_time
                
                # Update setpoint based on trajectory (position update loop)
                if self.trajectory and not self.trajectory.is_complete(current_time):
                    # Follow trajectory
                    trajectory_setpoint = self.trajectory.get_position(current_time)
                else:
                    # Trajectory complete, hold at target
                    trajectory_setpoint = self.target_setpoint
                
                # Update PID setpoint
                self.pid.set_setpoint(trajectory_setpoint, 0.0)
                
                # Compute control output (PID loop)
                control_output_x, control_output_y, saturated_x, saturated_y = self.pid.update(position_x, position_y)
                
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

