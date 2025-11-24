# Stewart Platform Controller
# Main controller integrating 2D ball detection and PID control for Stewart platform
# Adapted from 1D beam balancer for 2D platform control

import cv2
import numpy as np
import json
import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from threading import Thread
import queue
from ball_detection_2d import BallDetector2D
from pid_controller_2d import PIDController2D
from inverse_kinematics import StewartPlatformIK

class StewartPlatformController:
    """Main controller for Stewart platform ball balancing system."""
    
    def __init__(self, config_file="config_stewart.json"):
        """Initialize controller, load config, set defaults and queues."""
        # Load experiment and hardware config from JSON file
        try:
            with open(config_file, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            print(f"[WARNING] Config file {config_file} not found, using defaults")
            self.config = self._get_default_config()
        
        # Initialize 2D PID controller
        # X-axis (roll) PID gains
        Kp_x = self.config.get('pid', {}).get('Kp_x', 10.0)
        Ki_x = self.config.get('pid', {}).get('Ki_x', 0.0)
        Kd_x = self.config.get('pid', {}).get('Kd_x', 0.0)
        
        # Y-axis (pitch) PID gains
        Kp_y = self.config.get('pid', {}).get('Kp_y', 10.0)
        Ki_y = self.config.get('pid', {}).get('Ki_y', 0.0)
        Kd_y = self.config.get('pid', {}).get('Kd_y', 0.0)
        
        self.pid = PIDController2D(
            Kp_x=Kp_x, Ki_x=Ki_x, Kd_x=Kd_x,
            Kp_y=Kp_y, Ki_y=Ki_y, Kd_y=Kd_y,
            output_limit_x=self.config.get('platform', {}).get('max_roll_angle', 15.0),
            output_limit_y=self.config.get('platform', {}).get('max_pitch_angle', 15.0)
        )
        
        # Initialize ball detector
        self.detector = BallDetector2D(config_file)
        
        # Initialize inverse kinematics solver
        self.ik_solver = StewartPlatformIK(self.config)
        
        # Servo configuration (for 3 motors using Adafruit PWM Servo Driver on Arduino)
        # Single Arduino controls all 3 servos via I2C PWM driver
        servo_config = self.config.get('servo', {})
        self.servo_port = servo_config.get('port', "COM3")
        self.servo_baud_rate = servo_config.get('baud_rate', 115200)
        self.servo_read_timeout = max(servo_config.get('timeout_seconds', 1.0), 0.0)
        write_timeout_ms = servo_config.get('write_timeout_ms', 50)
        if write_timeout_ms is None:
            write_timeout_ms = 50
        self.servo_write_timeout = max(write_timeout_ms, 0) / 1000.0
        self.neutral_angles = servo_config.get('neutral_angles', [15, 15, 15])
        # Motor direction inversion: [False, False, False] means all motors normal direction
        # Set to True for any motor that spins the wrong way
        self.motor_direction_invert = servo_config.get('motor_direction_invert', [False, False, False])
        self.servo_serial = None
        
        # Use inverse kinematics flag (can be toggled)
        # Default to False to use simplified mapping (more reliable)
        self.use_inverse_kinematics = self.config.get('platform', {}).get('use_inverse_kinematics', False)
        
        # Controller-internal state
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        
        # Data logs for plotting results
        self.time_log = []
        self.position_x_log = []
        self.position_y_log = []
        self.setpoint_x_log = []
        self.setpoint_y_log = []
        self.control_x_log = []
        self.control_y_log = []
        self.start_time = None
        self.latest_position = (0.0, 0.0)
        self.latest_error = (0.0, 0.0)
        self.telemetry_window = None
        self.telemetry_canvas = None
        self.position_display_range = self._determine_position_range()
        self.telemetry_canvas_size = 360
        
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False    # Main run flag for clean shutdown
        
        # Square pattern state machine
        self.square_pattern_enabled = False
        self.square_pattern_index = 0
        # Default square pattern (can be overridden by user input)
        self.square_setpoints = [(0.0, 0.0), (0.05, 0.05), (0.05, -0.05), (-0.05, -0.05), (-0.05, 0.05)]
        self.settle_tolerance = 0.01  # position tolerance in meters (default)
        self.settle_duration = 2.0  # time in seconds ball must be within tolerance (default)
        self.settle_start_time = None  # timestamp when ball entered tolerance zone
        self.settled = False  # current settle status
        # Custom setpoint input storage (up to 10 points)
        self.custom_setpoint_entries = []  # Will store entry widgets
    
    def _get_default_config(self):
        """Return default configuration dictionary."""
        return {
            'camera': {
                'index': 0,
                'frame_width': 640,
                'frame_height': 480
            },
            'pid': {
                'Kp_x': 3.0,
                'Ki_x': 0.0,
                'Kd_x': 2.0,
                'Kp_y': 3.0,
                'Ki_y': 0.0,
                'Kd_y': 2.0
            },
            'platform': {
                'max_roll_angle': 15.0,
                'max_pitch_angle': 15.0
            },
            'servo': {
                'port': "COM3",
                'neutral_angles': [15, 15, 15],
                'motor_direction_invert': [False, False, False]  # Set to True for motors spinning wrong way
            }
        }
    
    def connect_servos(self):
        """Try to open serial connection to Arduino with PWM Servo Driver, return True if succeeds."""
        try:
            self.servo_serial = serial.Serial(
                self.servo_port,
                self.servo_baud_rate,
                timeout=self.servo_read_timeout,
                write_timeout=self.servo_write_timeout
            )
            time.sleep(2)  # Wait for Arduino to initialize
            # Flush any initial data
            self.servo_serial.reset_input_buffer()
            print(f"[ARDUINO] Connected to {self.servo_port}")
            return True
        except Exception as e:
            print(f"[ARDUINO] Failed to connect to {self.servo_port}: {e}")
            self.servo_serial = None
            return False
    
    def send_platform_tilt(self, roll_angle, pitch_angle):
        """Send tilt angles to platform motors using inverse kinematics.
        
        For a circular Stewart platform with 3 motors arranged at 120° intervals,
        we convert roll and pitch angles to motor commands using inverse kinematics.
        
        Args:
            roll_angle: Desired roll angle in degrees (rotation around X axis, positive = tilt right)
            pitch_angle: Desired pitch angle in degrees (rotation around Y axis, positive = tilt forward)
        """
        # Clip angles to safe range
        roll_angle = np.clip(roll_angle, -15, 15)
        pitch_angle = np.clip(pitch_angle, -15, 15)
        
        if self.use_inverse_kinematics:
            # Use inverse kinematics to calculate motor angles
            try:
                # Get motor joint angles from IK solver
                theta_11, theta_21, theta_31 = self.ik_solver.get_motor_angles(roll_angle, pitch_angle)
                
                # Convert joint angles to servo angles
                # The IK solver returns angles in degrees, we need to map them to servo range (0-30)
                # This mapping depends on your physical setup - adjust as needed
                
                # Option 1: Use theta_11, theta_21, theta_31 directly (if they're already in servo range)
                # Option 2: Map from joint angle range to servo range
                # For now, we'll use a simple offset and scaling
                
                # Get angle range from config or use defaults
                angle_scale = self.config.get('platform', {}).get('ik_angle_scale', 1.0)
                angle_offset = self.config.get('platform', {}).get('ik_angle_offset', 0.0)
                
                # Map IK angles to servo angles
                # Adjust these formulas based on your physical setup
                # Apply direction inversion if configured
                motor1_dir = -1.0 if self.motor_direction_invert[0] else 1.0
                motor2_dir = -1.0 if self.motor_direction_invert[1] else 1.0
                motor3_dir = -1.0 if self.motor_direction_invert[2] else 1.0
                
                motor1_angle = self.neutral_angles[0] + (theta_11 * angle_scale + angle_offset) * motor1_dir
                motor2_angle = self.neutral_angles[1] + (theta_21 * angle_scale + angle_offset) * motor2_dir
                motor3_angle = self.neutral_angles[2] + (theta_31 * angle_scale + angle_offset) * motor3_dir
                
            except Exception as e:
                print(f"[IK] Error in inverse kinematics: {e}, falling back to simplified method")
                # Fall back to simplified method on error
                motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(roll_angle, pitch_angle)
        else:
            # Use simplified trigonometric mapping (fallback)
            motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(roll_angle, pitch_angle)
        
        # Clip to servo range (0-30 degrees as per Arduino code)
        motor1_angle = int(np.clip(motor1_angle, 0, 30))
        motor2_angle = int(np.clip(motor2_angle, 0, 30))
        motor3_angle = int(np.clip(motor3_angle, 0, 30))
        
        # DEBUG: Print motor angles being sent
        print(f"[MOTOR] Roll={roll_angle:.1f}°, Pitch={pitch_angle:.1f}° -> "
              f"M1={motor1_angle}°, M2={motor2_angle}°, M3={motor3_angle}°")
        
        # Send all 3 servo commands to Arduino as 3 bytes
        # Arduino expects: byte1 (servo1), byte2 (servo2), byte3 (servo3)
        if self.servo_serial:
            try:
                self.servo_serial.write(bytes([motor1_angle, motor2_angle, motor3_angle]))
                self.servo_serial.flush()  # Ensure data is sent immediately
            except serial.SerialTimeoutException:
                print("[ARDUINO] Send timed out - Arduino likely busy printing serial logs. "
                      "Disable firmware debugging or raise baud rate.")
            except serial.SerialException as e:
                print(f"[ARDUINO] Serial error: {e}")
            except Exception as e:
                print(f"[ARDUINO] Send failed: {e}")
        else:
            print(f"[MOTOR] WARNING: No servo connection, not sending commands")
    
    def _simplified_motor_mapping(self, roll_angle, pitch_angle):
        """Simplified trigonometric mapping (fallback method).
        
        Uses calibrated motor angles from 3-point calibration if available,
        otherwise falls back to default angles.
        
        Args:
            roll_angle: Roll angle in degrees
            pitch_angle: Pitch angle in degrees
            
        Returns:
            tuple: (motor1_angle, motor2_angle, motor3_angle) in degrees
        """
        # Get motor angles from calibration if available
        motor_angles_deg = self.config.get('motor_angles_deg', None)
        
        if motor_angles_deg and len(motor_angles_deg) == 3:
            # Use calibrated motor angles (absolute angles from 3-point calibration)
            # These angles are already calculated relative to the platform center
            motor1_angle_deg = motor_angles_deg[0]-180
            motor2_angle_deg = motor_angles_deg[1]-180
            motor3_angle_deg = motor_angles_deg[2]-180
        else:
            # Fallback to default angles (120° spacing, starting at -90°)
            motor1_angle_deg = -90
            motor2_angle_deg = -210
            motor3_angle_deg = -330
        
        # Convert to radians
        motor1_angle_rad = np.radians(motor1_angle_deg)
        motor2_angle_rad = np.radians(motor2_angle_deg)
        motor3_angle_rad = np.radians(motor3_angle_deg)
        
        # Calculate height changes for each motor
        # Negative signs are needed for correct platform tilt direction
        motor1_height = -roll_angle * np.cos(motor1_angle_rad) - pitch_angle * np.sin(motor1_angle_rad)
        motor2_height = -roll_angle * np.cos(motor2_angle_rad) - pitch_angle * np.sin(motor2_angle_rad)
        motor3_height = -roll_angle * np.cos(motor3_angle_rad) - pitch_angle * np.sin(motor3_angle_rad)
        
        # Convert height changes to motor angles
        # Scale factor: platform tilt degrees -> servo angle change
        scale_factor = self.config.get('platform', {}).get('motor_scale_factor', 1.0)
        
        # Apply direction inversion if configured (multiply by -1 if inverted)
        motor1_dir = -1.0 if self.motor_direction_invert[0] else 1.0
        motor2_dir = -1.0 if self.motor_direction_invert[1] else 1.0
        motor3_dir = -1.0 if self.motor_direction_invert[2] else 1.0
        
        motor1_angle = self.neutral_angles[0] + motor1_height * scale_factor * motor1_dir
        motor2_angle = self.neutral_angles[1] + motor2_height * scale_factor * motor2_dir
        motor3_angle = self.neutral_angles[2] + motor3_height * scale_factor * motor3_dir
        
        # DEBUG to show intermediate calculations
        if abs(roll_angle) > 0.1 or abs(pitch_angle) > 0.1:
            print(f"[MAPPING] Roll={roll_angle:.1f}°, Pitch={pitch_angle:.1f}° -> "
                  f"Heights: M1={motor1_height:.2f}, M2={motor2_height:.2f}, M3={motor3_height:.2f} -> "
                  f"Angles: M1={motor1_angle:.1f}°, M2={motor2_angle:.1f}°, M3={motor3_angle:.1f}°")
        
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
            
            # Detect ball position in frame (2D)
            found, center, radius, position_x_m, position_y_m = self.detector.detect_ball(frame)
            
            if found:
                # Always keep latest measurement only
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait((position_x_m, position_y_m))
                except Exception:
                    pass
            
            # Show processed video with overlays
            vis_frame, _, _, _ = self.detector.draw_detection(frame)
            cv2.imshow("Stewart Platform - Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    def control_thread(self):
        """Runs PID control loop in parallel with GUI and camera."""
        if not self.connect_servos():
            print("[WARNING] No servos connected - running in simulation mode")
        
        self.start_time = time.time()
        self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
        
        while self.running:
            try:
                # Wait for latest ball position from camera
                position_x, position_y = self.position_queue.get(timeout=0.1)
                
                # Handle square pattern if enabled
                if self.square_pattern_enabled:
                    # Check if ball has settled at current setpoint
                    if self._check_settled(position_x, position_y):
                        # Advance to next setpoint in sequence
                        self.square_pattern_index = (self.square_pattern_index + 1) % len(self.square_setpoints)
                        sp_x, sp_y = self.square_setpoints[self.square_pattern_index]
                        self.setpoint_x = sp_x
                        self.setpoint_y = sp_y
                        self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
                        # Reset settle tracking for new setpoint
                        self.settle_start_time = None
                        self.settled = False
                        print(f"[PATTERN] Advanced to setpoint {self.square_pattern_index + 1}/{len(self.square_setpoints)}: ({sp_x:.3f}, {sp_y:.3f})")
                
                # Compute control output using 2D PID
                control_output_x, control_output_y = self.pid.update(position_x, position_y)
                
                # Send control command to platform (real or simulated)
                self.send_platform_tilt(control_output_x, control_output_y)
                
                # Log results for plotting
                current_time = time.time() - self.start_time
                self.time_log.append(current_time)
                self.position_x_log.append(position_x)
                self.position_y_log.append(position_y)
                self.setpoint_x_log.append(self.setpoint_x)
                self.setpoint_y_log.append(self.setpoint_y)
                self.control_x_log.append(control_output_x)
                self.control_y_log.append(control_output_y)
                
                print(f"Pos: X={position_x:.3f}m, Y={position_y:.3f}m | "
                      f"PID Output: Roll={control_output_x:.1f}°, Pitch={control_output_y:.1f}°")
                self.latest_position = (position_x, position_y)
                self.latest_error = (self.setpoint_x - position_x, self.setpoint_y - position_y)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break
        
        # Return to neutral on exit
        self.send_platform_tilt(0, 0)
        if self.servo_serial:
            self.servo_serial.close()
    
    def _check_settled(self, position_x, position_y):
        """Check if ball has settled at current setpoint.
        
        Args:
            position_x: Current X position (meters)
            position_y: Current Y position (meters)
            
        Returns:
            bool: True if ball has been within tolerance for settle_duration seconds
        """
        # Calculate distance from setpoint
        error_x = abs(self.setpoint_x - position_x)
        error_y = abs(self.setpoint_y - position_y)
        
        # Check if within tolerance
        within_tolerance = (error_x <= self.settle_tolerance and 
                          error_y <= self.settle_tolerance)
        
        current_time = time.time()
        
        if within_tolerance:
            # If we just entered tolerance zone, start timer
            if self.settle_start_time is None:
                self.settle_start_time = current_time
                self.settled = False
            # Check if we've been in tolerance long enough
            elif current_time - self.settle_start_time >= self.settle_duration:
                self.settled = True
            else:
                self.settled = False
        else:
            # Outside tolerance, reset timer
            self.settle_start_time = None
            self.settled = False
        
        return self.settled
    
    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Stewart Platform PID Controller")
        self.root.geometry("600x950")
        
        # Title label
        ttk.Label(self.root, text="Stewart Platform Control", font=("Arial", 18, "bold")).pack(pady=10)
        
        # X-axis (Roll) controls
        ttk.Label(self.root, text="X-Axis (Roll) PID Gains", font=("Arial", 14, "bold")).pack(pady=5)
        
        # Kp_x slider with entry box
        ttk.Label(self.root, text="Kp (Proportional)", font=("Arial", 10)).pack()
        kp_x_frame = ttk.Frame(self.root)
        kp_x_frame.pack(pady=2)
        self.kp_x_var = tk.DoubleVar(value=self.pid.Kp_x)
        kp_x_slider = ttk.Scale(kp_x_frame, from_=0, to=100, variable=self.kp_x_var,
                               orient=tk.HORIZONTAL, length=450)
        kp_x_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.kp_x_entry = ttk.Entry(kp_x_frame, width=10)
        self.kp_x_entry.insert(0, f"{self.pid.Kp_x:.2f}")
        self.kp_x_entry.pack(side=tk.LEFT)
        self.kp_x_entry.bind('<Return>', lambda e: self._update_from_entry(self.kp_x_var, self.kp_x_entry))
        self.kp_x_label = ttk.Label(self.root, text=f"Kp_x: {self.pid.Kp_x:.1f}", font=("Arial", 9))
        self.kp_x_label.pack()
        
        # Ki_x slider with entry box
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 10)).pack()
        ki_x_frame = ttk.Frame(self.root)
        ki_x_frame.pack(pady=2)
        self.ki_x_var = tk.DoubleVar(value=self.pid.Ki_x)
        ki_x_slider = ttk.Scale(ki_x_frame, from_=0, to=10, variable=self.ki_x_var,
                               orient=tk.HORIZONTAL, length=450)
        ki_x_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.ki_x_entry = ttk.Entry(ki_x_frame, width=10)
        self.ki_x_entry.insert(0, f"{self.pid.Ki_x:.2f}")
        self.ki_x_entry.pack(side=tk.LEFT)
        self.ki_x_entry.bind('<Return>', lambda e: self._update_from_entry(self.ki_x_var, self.ki_x_entry))
        self.ki_x_label = ttk.Label(self.root, text=f"Ki_x: {self.pid.Ki_x:.1f}", font=("Arial", 9))
        self.ki_x_label.pack()
        
        # Kd_x slider with entry box
        ttk.Label(self.root, text="Kd (Derivative)", font=("Arial", 10)).pack()
        kd_x_frame = ttk.Frame(self.root)
        kd_x_frame.pack(pady=2)
        self.kd_x_var = tk.DoubleVar(value=self.pid.Kd_x)
        kd_x_slider = ttk.Scale(kd_x_frame, from_=0, to=20, variable=self.kd_x_var,
                               orient=tk.HORIZONTAL, length=450)
        kd_x_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.kd_x_entry = ttk.Entry(kd_x_frame, width=10)
        self.kd_x_entry.insert(0, f"{self.pid.Kd_x:.2f}")
        self.kd_x_entry.pack(side=tk.LEFT)
        self.kd_x_entry.bind('<Return>', lambda e: self._update_from_entry(self.kd_x_var, self.kd_x_entry))
        self.kd_x_label = ttk.Label(self.root, text=f"Kd_x: {self.pid.Kd_x:.1f}", font=("Arial", 9))
        self.kd_x_label.pack()
        
        # Y-axis (Pitch) controls
        ttk.Label(self.root, text="Y-Axis (Pitch) PID Gains", font=("Arial", 14, "bold")).pack(pady=5)
        
        # Kp_y slider with entry box
        ttk.Label(self.root, text="Kp (Proportional)", font=("Arial", 10)).pack()
        kp_y_frame = ttk.Frame(self.root)
        kp_y_frame.pack(pady=2)
        self.kp_y_var = tk.DoubleVar(value=self.pid.Kp_y)
        kp_y_slider = ttk.Scale(kp_y_frame, from_=0, to=100, variable=self.kp_y_var,
                               orient=tk.HORIZONTAL, length=450)
        kp_y_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.kp_y_entry = ttk.Entry(kp_y_frame, width=10)
        self.kp_y_entry.insert(0, f"{self.pid.Kp_y:.2f}")
        self.kp_y_entry.pack(side=tk.LEFT)
        self.kp_y_entry.bind('<Return>', lambda e: self._update_from_entry(self.kp_y_var, self.kp_y_entry))
        self.kp_y_label = ttk.Label(self.root, text=f"Kp_y: {self.pid.Kp_y:.1f}", font=("Arial", 9))
        self.kp_y_label.pack()
        
        # Ki_y slider with entry box
        ttk.Label(self.root, text="Ki (Integral)", font=("Arial", 10)).pack()
        ki_y_frame = ttk.Frame(self.root)
        ki_y_frame.pack(pady=2)
        self.ki_y_var = tk.DoubleVar(value=self.pid.Ki_y)
        ki_y_slider = ttk.Scale(ki_y_frame, from_=0, to=10, variable=self.ki_y_var,
                               orient=tk.HORIZONTAL, length=450)
        ki_y_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.ki_y_entry = ttk.Entry(ki_y_frame, width=10)
        self.ki_y_entry.insert(0, f"{self.pid.Ki_y:.2f}")
        self.ki_y_entry.pack(side=tk.LEFT)
        self.ki_y_entry.bind('<Return>', lambda e: self._update_from_entry(self.ki_y_var, self.ki_y_entry))
        self.ki_y_label = ttk.Label(self.root, text=f"Ki_y: {self.pid.Ki_y:.1f}", font=("Arial", 9))
        self.ki_y_label.pack()
        
        # Kd_y slider with entry box
        ttk.Label(self.root, text="Kd (Derivative)", font=("Arial", 10)).pack()
        kd_y_frame = ttk.Frame(self.root)
        kd_y_frame.pack(pady=2)
        self.kd_y_var = tk.DoubleVar(value=self.pid.Kd_y)
        kd_y_slider = ttk.Scale(kd_y_frame, from_=0, to=20, variable=self.kd_y_var,
                               orient=tk.HORIZONTAL, length=450)
        kd_y_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.kd_y_entry = ttk.Entry(kd_y_frame, width=10)
        self.kd_y_entry.insert(0, f"{self.pid.Kd_y:.2f}")
        self.kd_y_entry.pack(side=tk.LEFT)
        self.kd_y_entry.bind('<Return>', lambda e: self._update_from_entry(self.kd_y_var, self.kd_y_entry))
        self.kd_y_label = ttk.Label(self.root, text=f"Kd_y: {self.pid.Kd_y:.1f}", font=("Arial", 9))
        self.kd_y_label.pack()
        
        # Square Pattern controls
        ttk.Label(self.root, text="Custom Pattern (up to 10 setpoints)", font=("Arial", 12, "bold")).pack(pady=5)
        
        # Setpoint input section
        setpoints_frame = ttk.LabelFrame(self.root, text="Setpoints", padding=5)
        setpoints_frame.pack(pady=5, padx=10, fill=tk.BOTH, expand=True)
        
        # Header row
        header_frame = ttk.Frame(setpoints_frame)
        header_frame.pack(fill=tk.X, pady=2)
        ttk.Label(header_frame, text="#", font=("Arial", 9, "bold"), width=3).pack(side=tk.LEFT, padx=2)
        ttk.Label(header_frame, text="X (m)", font=("Arial", 9, "bold"), width=10).pack(side=tk.LEFT, padx=2)
        ttk.Label(header_frame, text="Y (m)", font=("Arial", 9, "bold"), width=10).pack(side=tk.LEFT, padx=2)
        
        # Create entry fields for up to 10 setpoints
        self.custom_setpoint_entries = []
        for i in range(10):
            row_frame = ttk.Frame(setpoints_frame)
            row_frame.pack(fill=tk.X, pady=1)
            ttk.Label(row_frame, text=f"{i+1}", font=("Arial", 9), width=3).pack(side=tk.LEFT, padx=2)
            x_entry = ttk.Entry(row_frame, width=10)
            x_entry.pack(side=tk.LEFT, padx=2)
            y_entry = ttk.Entry(row_frame, width=10)
            y_entry.pack(side=tk.LEFT, padx=2)
            self.custom_setpoint_entries.append((x_entry, y_entry))
        
        # Initialize with default square pattern
        default_setpoints = [(0.0, 0.0), (0.05, 0.05), (0.05, -0.05), (-0.05, -0.05), (-0.05, 0.05)]
        for i, (x, y) in enumerate(default_setpoints):
            if i < len(self.custom_setpoint_entries):
                self.custom_setpoint_entries[i][0].insert(0, f"{x:.4f}")
                self.custom_setpoint_entries[i][1].insert(0, f"{y:.4f}")
        
        # Pattern control buttons
        pattern_control_frame = ttk.Frame(self.root)
        pattern_control_frame.pack(pady=5)
        ttk.Button(pattern_control_frame, text="Update Pattern",
                  command=self.update_pattern_from_entries).pack(side=tk.LEFT, padx=5)
        ttk.Button(pattern_control_frame, text="Clear All",
                  command=self.clear_all_setpoints).pack(side=tk.LEFT, padx=5)
        ttk.Button(pattern_control_frame, text="Generate Circle (10cm)",
                  command=self.generate_circle_pattern).pack(side=tk.LEFT, padx=5)
        self.square_pattern_button = ttk.Button(pattern_control_frame, text="Start Pattern",
                                                command=self.toggle_square_pattern)
        self.square_pattern_button.pack(side=tk.LEFT, padx=5)
        
        # Settle parameters
        settle_params_frame = ttk.Frame(self.root)
        settle_params_frame.pack(pady=2)
        ttk.Label(settle_params_frame, text="Settle Tolerance (m):", font=("Arial", 9)).pack(side=tk.LEFT, padx=2)
        self.settle_tolerance_entry = ttk.Entry(settle_params_frame, width=8)
        self.settle_tolerance_entry.insert(0, f"{self.settle_tolerance:.4f}")
        self.settle_tolerance_entry.pack(side=tk.LEFT, padx=2)
        self.settle_tolerance_entry.bind('<Return>', lambda e: self._update_settle_tolerance())
        
        ttk.Label(settle_params_frame, text="Settle Duration (s):", font=("Arial", 9)).pack(side=tk.LEFT, padx=2)
        self.settle_duration_entry = ttk.Entry(settle_params_frame, width=8)
        self.settle_duration_entry.insert(0, f"{self.settle_duration:.2f}")
        self.settle_duration_entry.pack(side=tk.LEFT, padx=2)
        self.settle_duration_entry.bind('<Return>', lambda e: self._update_settle_duration())
        
        self.pattern_status_label = ttk.Label(self.root, text="Pattern: Stopped (0 setpoints)", font=("Arial", 9))
        self.pattern_status_label.pack(pady=2)
        
        # Setpoint controls
        ttk.Label(self.root, text="Setpoint", font=("Arial", 12)).pack(pady=5)
        
        # Setpoint X with entry box
        ttk.Label(self.root, text="Setpoint X (meters)", font=("Arial", 10)).pack()
        pos_range = 0.15  # Default range (matches platform radius)
        setpoint_x_frame = ttk.Frame(self.root)
        setpoint_x_frame.pack(pady=2)
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        self.setpoint_x_slider = ttk.Scale(setpoint_x_frame, from_=-pos_range, to=pos_range,
                                     variable=self.setpoint_x_var,
                                     orient=tk.HORIZONTAL, length=450)
        self.setpoint_x_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.setpoint_x_entry = ttk.Entry(setpoint_x_frame, width=10)
        self.setpoint_x_entry.insert(0, f"{self.setpoint_x:.4f}")
        self.setpoint_x_entry.pack(side=tk.LEFT)
        self.setpoint_x_entry.bind('<Return>', lambda e: self._update_from_entry(self.setpoint_x_var, self.setpoint_x_entry))
        self.setpoint_x_label = ttk.Label(self.root, text=f"Setpoint X: {self.setpoint_x:.4f}m", font=("Arial", 9))
        self.setpoint_x_label.pack()
        
        # Setpoint Y with entry box
        ttk.Label(self.root, text="Setpoint Y (meters)", font=("Arial", 10)).pack()
        setpoint_y_frame = ttk.Frame(self.root)
        setpoint_y_frame.pack(pady=2)
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)
        self.setpoint_y_slider = ttk.Scale(setpoint_y_frame, from_=-pos_range, to=pos_range,
                                     variable=self.setpoint_y_var,
                                     orient=tk.HORIZONTAL, length=450)
        self.setpoint_y_slider.pack(side=tk.LEFT, padx=(0, 5))
        self.setpoint_y_entry = ttk.Entry(setpoint_y_frame, width=10)
        self.setpoint_y_entry.insert(0, f"{self.setpoint_y:.4f}")
        self.setpoint_y_entry.pack(side=tk.LEFT)
        self.setpoint_y_entry.bind('<Return>', lambda e: self._update_from_entry(self.setpoint_y_var, self.setpoint_y_entry))
        self.setpoint_y_label = ttk.Label(self.root, text=f"Setpoint Y: {self.setpoint_y:.4f}m", font=("Arial", 9))
        self.setpoint_y_label.pack()
        
        # Button group for actions
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)
        ttk.Button(button_frame, text="Reset Integrals",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Show Telemetry",
                   command=self.show_telemetry_window).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)
        
        # Schedule periodic GUI update
        self.update_gui()
        # Launch telemetry window immediately
        self.show_telemetry_window()
    
    def _update_from_entry(self, var, entry):
        """Update a variable from an entry box when Enter is pressed."""
        try:
            value = float(entry.get())
            var.set(value)
        except ValueError:
            # If invalid input, reset entry to current var value
            entry.delete(0, tk.END)
            entry.insert(0, f"{var.get():.2f}")
    
    def _update_entry_from_var(self, entry, var, decimals=2):
        """Update entry box text from variable value (when not focused)."""
        current_text = entry.get()
        new_text = f"{var.get():.{decimals}f}"
        if current_text != new_text:
            entry.delete(0, tk.END)
            entry.insert(0, new_text)
    
    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            # PID parameters for X axis
            self.pid.set_gains_x(self.kp_x_var.get(), self.ki_x_var.get(), self.kd_x_var.get())
            # PID parameters for Y axis
            self.pid.set_gains_y(self.kp_y_var.get(), self.ki_y_var.get(), self.kd_y_var.get())
            
            # Setpoints (only update from GUI if pattern is not enabled)
            if not self.square_pattern_enabled:
                self.setpoint_x = self.setpoint_x_var.get()
                self.setpoint_y = self.setpoint_y_var.get()
                self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
            else:
                # Update GUI to reflect pattern setpoint
                self.setpoint_x_var.set(self.setpoint_x)
                self.setpoint_y_var.set(self.setpoint_y)
                # Update pattern status
                if len(self.square_setpoints) > 0:
                    self.pattern_status_label.config(
                        text=f"Pattern: Running - Setpoint {self.square_pattern_index + 1}/{len(self.square_setpoints)}: "
                        f"({self.setpoint_x:.3f}, {self.setpoint_y:.3f})")
            
            # Update displayed values
            self.kp_x_label.config(text=f"Kp_x: {self.pid.Kp_x:.1f}")
            self.ki_x_label.config(text=f"Ki_x: {self.pid.Ki_x:.1f}")
            self.kd_x_label.config(text=f"Kd_x: {self.pid.Kd_x:.1f}")
            self.kp_y_label.config(text=f"Kp_y: {self.pid.Kp_y:.1f}")
            self.ki_y_label.config(text=f"Ki_y: {self.pid.Ki_y:.1f}")
            self.kd_y_label.config(text=f"Kd_y: {self.pid.Kd_y:.1f}")
            self.setpoint_x_label.config(text=f"Setpoint X: {self.setpoint_x:.4f}m")
            self.setpoint_y_label.config(text=f"Setpoint Y: {self.setpoint_y:.4f}m")
            
            # Update entry boxes if they don't have focus (to avoid interfering with typing)
            if self.root.focus_get() != self.kp_x_entry:
                self._update_entry_from_var(self.kp_x_entry, self.kp_x_var, 2)
            if self.root.focus_get() != self.ki_x_entry:
                self._update_entry_from_var(self.ki_x_entry, self.ki_x_var, 2)
            if self.root.focus_get() != self.kd_x_entry:
                self._update_entry_from_var(self.kd_x_entry, self.kd_x_var, 2)
            if self.root.focus_get() != self.kp_y_entry:
                self._update_entry_from_var(self.kp_y_entry, self.kp_y_var, 2)
            if self.root.focus_get() != self.ki_y_entry:
                self._update_entry_from_var(self.ki_y_entry, self.ki_y_var, 2)
            if self.root.focus_get() != self.kd_y_entry:
                self._update_entry_from_var(self.kd_y_entry, self.kd_y_var, 2)
            if self.root.focus_get() != self.setpoint_x_entry:
                self._update_entry_from_var(self.setpoint_x_entry, self.setpoint_x_var, 4)
            if self.root.focus_get() != self.setpoint_y_entry:
                self._update_entry_from_var(self.setpoint_y_entry, self.setpoint_y_var, 4)
            
            self.update_telemetry()
            
            # Call again after 50 ms
            self.root.after(50, self.update_gui)
    
    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.pid.reset_integral()
    
    def toggle_square_pattern(self):
        """Toggle square pattern on/off."""
        if self.square_pattern_enabled:
            self.stop_square_pattern()
        else:
            self.start_square_pattern()
    
    def update_pattern_from_entries(self):
        """Update pattern setpoints from entry fields."""
        new_setpoints = []
        for x_entry, y_entry in self.custom_setpoint_entries:
            x_str = x_entry.get().strip()
            y_str = y_entry.get().strip()
            if x_str and y_str:
                try:
                    x_val = float(x_str)
                    y_val = float(y_str)
                    new_setpoints.append((x_val, y_val))
                except ValueError:
                    print(f"[PATTERN] Invalid setpoint entry: ({x_str}, {y_str})")
        
        if len(new_setpoints) == 0:
            print("[PATTERN] No valid setpoints entered. Pattern must have at least 1 setpoint.")
            self.pattern_status_label.config(text="Pattern: No valid setpoints (need at least 1)")
            return
        
        self.square_setpoints = new_setpoints
        self.square_pattern_index = 0  # Reset to first setpoint
        
        print(f"[PATTERN] Updated pattern with {len(new_setpoints)} setpoints: {new_setpoints}")
        
        # If pattern is running, update to new first setpoint
        if self.square_pattern_enabled:
            sp_x, sp_y = self.square_setpoints[0]
            self.setpoint_x = sp_x
            self.setpoint_y = sp_y
            self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
            self.settle_start_time = None
            self.settled = False
            self.pattern_status_label.config(
                text=f"Pattern: Running - Setpoint 1/{len(self.square_setpoints)}: ({sp_x:.3f}, {sp_y:.3f})")
        else:
            self.pattern_status_label.config(text=f"Pattern: Ready ({len(new_setpoints)} setpoints)")
    
    def clear_all_setpoints(self):
        """Clear all setpoint entry fields."""
        for x_entry, y_entry in self.custom_setpoint_entries:
            x_entry.delete(0, tk.END)
            y_entry.delete(0, tk.END)
        print("[PATTERN] Cleared all setpoint entries")
        self.pattern_status_label.config(text="Pattern: Stopped (0 setpoints)")
    
    def generate_circle_pattern(self):
        """Generate setpoints for a circle with 10 cm radius."""
        import math
        
        radius = 0.10  # 10 cm in meters
        num_points = 16  # Number of points around the circle
        center_x = 0.0
        center_y = 0.0
        
        # Clear all entries first
        for x_entry, y_entry in self.custom_setpoint_entries:
            x_entry.delete(0, tk.END)
            y_entry.delete(0, tk.END)
        
        # Generate circular setpoints
        circle_setpoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            circle_setpoints.append((x, y))
            
            # Fill in entry fields
            if i < len(self.custom_setpoint_entries):
                self.custom_setpoint_entries[i][0].insert(0, f"{x:.4f}")
                self.custom_setpoint_entries[i][1].insert(0, f"{y:.4f}")
        
        # Automatically update the pattern
        self.square_setpoints = circle_setpoints
        self.square_pattern_index = 0
        
        # If pattern is running, update to new first setpoint
        if self.square_pattern_enabled:
            sp_x, sp_y = self.square_setpoints[0]
            self.setpoint_x = sp_x
            self.setpoint_y = sp_y
            self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
            self.settle_start_time = None
            self.settled = False
            self.pattern_status_label.config(
                text=f"Pattern: Running - Setpoint 1/{len(self.square_setpoints)}: ({sp_x:.3f}, {sp_y:.3f})")
        else:
            self.pattern_status_label.config(text=f"Pattern: Ready ({len(circle_setpoints)} setpoints - Circle 10cm)")
        
        print(f"[PATTERN] Generated circle pattern with {num_points} points, radius {radius*100:.1f} cm")
    
    def start_square_pattern(self):
        """Start square pattern sequence."""
        # Make sure we have valid setpoints
        if len(self.square_setpoints) == 0:
            print("[PATTERN] No setpoints configured. Please enter setpoints and click 'Update Pattern' first.")
            self.pattern_status_label.config(text="Pattern: Error - No setpoints configured")
            return
        
        self.square_pattern_enabled = True
        self.square_pattern_index = 0
        sp_x, sp_y = self.square_setpoints[self.square_pattern_index]
        self.setpoint_x = sp_x
        self.setpoint_y = sp_y
        self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
        self.settle_start_time = None
        self.settled = False
        
        # Update GUI
        self.square_pattern_button.config(text="Stop Pattern")
        self.pattern_status_label.config(text=f"Pattern: Running - Setpoint 1/{len(self.square_setpoints)}: ({sp_x:.3f}, {sp_y:.3f})")
        
        # Disable setpoint sliders when pattern is active
        if hasattr(self, 'setpoint_x_slider'):
            self.setpoint_x_slider.config(state='disabled')
        if hasattr(self, 'setpoint_y_slider'):
            self.setpoint_y_slider.config(state='disabled')
        
        print(f"[PATTERN] Started pattern with {len(self.square_setpoints)} setpoints, starting at ({sp_x:.3f}, {sp_y:.3f})")
    
    def stop_square_pattern(self):
        """Stop square pattern sequence."""
        self.square_pattern_enabled = False
        self.settle_start_time = None
        self.settled = False
        
        # Update GUI
        self.square_pattern_button.config(text="Start Pattern")
        num_setpoints = len(self.square_setpoints)
        self.pattern_status_label.config(text=f"Pattern: Stopped ({num_setpoints} setpoints)")
        
        # Re-enable setpoint sliders
        if hasattr(self, 'setpoint_x_slider'):
            self.setpoint_x_slider.config(state='normal')
        if hasattr(self, 'setpoint_y_slider'):
            self.setpoint_y_slider.config(state='normal')
        
        print("[PATTERN] Stopped pattern")
    
    def _update_settle_tolerance(self):
        """Update settle tolerance from entry box."""
        try:
            value = float(self.settle_tolerance_entry.get())
            if value > 0:
                self.settle_tolerance = value
                print(f"[PATTERN] Settle tolerance updated to {value:.4f}m")
            else:
                # Reset to current value if invalid
                self.settle_tolerance_entry.delete(0, tk.END)
                self.settle_tolerance_entry.insert(0, f"{self.settle_tolerance:.4f}")
        except ValueError:
            # Reset to current value if invalid
            self.settle_tolerance_entry.delete(0, tk.END)
            self.settle_tolerance_entry.insert(0, f"{self.settle_tolerance:.4f}")
    
    def _update_settle_duration(self):
        """Update settle duration from entry box."""
        try:
            value = float(self.settle_duration_entry.get())
            if value > 0:
                self.settle_duration = value
                print(f"[PATTERN] Settle duration updated to {value:.2f}s")
            else:
                # Reset to current value if invalid
                self.settle_duration_entry.delete(0, tk.END)
                self.settle_duration_entry.insert(0, f"{self.settle_duration:.2f}")
        except ValueError:
            # Reset to current value if invalid
            self.settle_duration_entry.delete(0, tk.END)
            self.settle_duration_entry.insert(0, f"{self.settle_duration:.2f}")
    
    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        
        fig = plt.figure(figsize=(12, 12))
        gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.1])
        
        ax_x = fig.add_subplot(gs[0, 0])
        ax_y = fig.add_subplot(gs[0, 1])
        ax_roll = fig.add_subplot(gs[1, 0])
        ax_pitch = fig.add_subplot(gs[1, 1])
        ax_plane = fig.add_subplot(gs[2, :])
        
        # X position trace
        ax_x.plot(self.time_log, self.position_x_log, label="Ball X Position", linewidth=2)
        ax_x.plot(self.time_log, self.setpoint_x_log, label="Setpoint X",
                  linestyle="--", linewidth=2)
        ax_x.set_ylabel("Position X (m)")
        ax_x.set_title("X-Axis (Roll) Control")
        ax_x.legend()
        ax_x.grid(True, alpha=0.3)
        
        # Y position trace
        ax_y.plot(self.time_log, self.position_y_log, label="Ball Y Position", linewidth=2, color='green')
        ax_y.plot(self.time_log, self.setpoint_y_log, label="Setpoint Y",
                  linestyle="--", linewidth=2)
        ax_y.set_ylabel("Position Y (m)")
        ax_y.set_title("Y-Axis (Pitch) Control")
        ax_y.legend()
        ax_y.grid(True, alpha=0.3)
        
        # X control output trace
        ax_roll.plot(self.time_log, self.control_x_log, label="Roll Output",
                     color="orange", linewidth=2)
        ax_roll.set_xlabel("Time (s)")
        ax_roll.set_ylabel("Roll Angle (degrees)")
        ax_roll.legend()
        ax_roll.grid(True, alpha=0.3)
        
        # Y control output trace
        ax_pitch.plot(self.time_log, self.control_y_log, label="Pitch Output",
                      color="red", linewidth=2)
        ax_pitch.set_xlabel("Time (s)")
        ax_pitch.set_ylabel("Pitch Angle (degrees)")
        ax_pitch.legend()
        ax_pitch.grid(True, alpha=0.3)
        
        # Plane plot
        radius = self.position_display_range
        theta = np.linspace(0, 2 * np.pi, 400)
        ax_plane.plot(radius * np.cos(theta), radius * np.sin(theta),
                      color="#888888", linestyle="--", label="Plate Boundary")
        ax_plane.plot(self.position_x_log, self.position_y_log,
                      label="Ball Path", linewidth=2, color="#1f77b4")
        ax_plane.scatter(self.setpoint_x_log[-1], self.setpoint_y_log[-1],
                         marker="x", color="red", s=80, label="Target")
        ax_plane.scatter(self.position_x_log[-1], self.position_y_log[-1],
                         marker="o", color="green", s=50, label="Last Position")
        ax_plane.set_xlabel("X (m)")
        ax_plane.set_ylabel("Y (m)")
        ax_plane.set_title("Platform Plane")
        ax_plane.set_xlim(-radius, radius)
        ax_plane.set_ylim(-radius, radius)
        ax_plane.set_aspect('equal', adjustable='box')
        ax_plane.grid(True, alpha=0.2)
        ax_plane.legend(loc="upper right")
        
        plt.tight_layout()
        plt.show()
    
    def stop(self):
        """Stop everything and clean up threads and GUI."""
        self.running = False
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass
        self._close_telemetry_window()
    
    def run(self):
        """Entry point: starts threads, launches GUI mainloop."""
        print("[INFO] Starting Stewart Platform Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        self.running = True
        
        # Start camera and control threads, mark as daemon for exit
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()
        
        # Build and run GUI in main thread
        self.create_gui()
        self.root.mainloop()
        
        # After GUI ends, stop everything
        self.running = False
        print("[INFO] Controller stopped")

    def _determine_position_range(self):
        """Determine display radius for telemetry plots."""
        calibration = self.config.get('calibration', {})
        platform_radius = (self.config.get('platform_radius_m') or
                           self.config.get('platform', {}).get('platform_radius_m') or
                           0.0)

        def safe_abs(value):
            return abs(value) if isinstance(value, (int, float)) else 0.0

        candidates = [
            safe_abs(calibration.get('position_max_x_m')),
            safe_abs(calibration.get('position_min_x_m')),
            safe_abs(calibration.get('position_max_y_m')),
            safe_abs(calibration.get('position_min_y_m')),
            safe_abs(platform_radius)
        ]
        max_candidate = max([c for c in candidates if c], default=0.1)
        return max(0.05, float(max_candidate))

    def show_telemetry_window(self):
        """Ensure telemetry window is visible."""
        if self.telemetry_window is None or not self.telemetry_window.winfo_exists():
            self.create_telemetry_window()
        else:
            self.telemetry_window.deiconify()
            self.telemetry_window.lift()

    def create_telemetry_window(self):
        """Create telemetry overlay window with live data."""
        self.telemetry_window = tk.Toplevel(self.root)
        self.telemetry_window.title("Telemetry - Position vs Setpoint")
        self.telemetry_window.geometry("420x520")
        self.telemetry_window.protocol("WM_DELETE_WINDOW", self._close_telemetry_window)
        
        self.telemetry_actual_label = ttk.Label(self.telemetry_window, text="Ball: X=0.000m, Y=0.000m",
                                                font=("Consolas", 12))
        self.telemetry_actual_label.pack(pady=5)
        self.telemetry_target_label = ttk.Label(self.telemetry_window, text="Setpoint: X=0.000m, Y=0.000m",
                                                font=("Consolas", 12))
        self.telemetry_target_label.pack(pady=5)
        self.telemetry_error_label = ttk.Label(self.telemetry_window, text="Error: X=0.000m, Y=0.000m",
                                               font=("Consolas", 12, "bold"))
        self.telemetry_error_label.pack(pady=5)
        
        self.telemetry_canvas = tk.Canvas(self.telemetry_window,
                                          width=self.telemetry_canvas_size,
                                          height=self.telemetry_canvas_size,
                                          bg="#111111", highlightthickness=0)
        self.telemetry_canvas.pack(pady=10)
        self.update_telemetry(force=True)

    def _close_telemetry_window(self):
        """Handle telemetry window close event."""
        if self.telemetry_window is not None:
            try:
                self.telemetry_window.destroy()
            except Exception:
                pass
        self.telemetry_window = None
        self.telemetry_canvas = None

    def update_telemetry(self, force=False):
        """Update telemetry labels and canvas."""
        if (self.telemetry_window is None or
                self.telemetry_canvas is None or
                not self.telemetry_window.winfo_exists()):
            if force:
                self.show_telemetry_window()
            return
        
        x, y = self.latest_position
        err_x, err_y = self.latest_error
        
        self.telemetry_actual_label.config(text=f"Ball:     X={x:+0.4f} m  |  Y={y:+0.4f} m")
        self.telemetry_target_label.config(text=f"Setpoint: X={self.setpoint_x:+0.4f} m  |  Y={self.setpoint_y:+0.4f} m")
        self.telemetry_error_label.config(text=f"Error:    X={err_x:+0.4f} m  |  Y={err_y:+0.4f} m")
        
        canvas = self.telemetry_canvas
        canvas.delete("all")
        size = self.telemetry_canvas_size
        center = size / 2
        radius_px = center - 20
        
        # Draw plate boundary
        canvas.create_oval(center - radius_px, center - radius_px,
                           center + radius_px, center + radius_px,
                           outline="#666", width=2)
        canvas.create_line(center, 10, center, size - 10, fill="#333")
        canvas.create_line(10, center, size - 10, center, fill="#333")
        
        def to_canvas(px, py):
            scale = radius_px / self.position_display_range
            limit = self.position_display_range * 1.1
            px = max(min(px, limit), -limit)
            py = max(min(py, limit), -limit)
            cx = center + px * scale
            cy = center - py * scale
            return cx, cy
        
        # Draw setpoint and actual positions
        spx, spy = to_canvas(self.setpoint_x, self.setpoint_y)
        bx, by = to_canvas(x, y)
        canvas.create_oval(spx - 6, spy - 6, spx + 6, spy + 6,
                           outline="#ff5555", width=2)
        canvas.create_oval(bx - 6, by - 6, bx + 6, by + 6,
                           fill="#55ff55", outline="")
        canvas.create_line(spx, spy, bx, by, fill="#ffaa00", dash=(3, 3))
        
        # Draw motor positions if available in config
        motor_positions_pixels = self.config.get('motor_positions_pixels', None)
        platform_center_pixels = self.config.get('platform_center_pixels', None)
        pixel_to_meter_ratio = self.config.get('calibration', {}).get('pixel_to_meter_ratio', None)
        
        if motor_positions_pixels and platform_center_pixels and pixel_to_meter_ratio:
            motor_colors = ["#ff00ff", "#00ffff", "#ffff00"]  # Magenta, Cyan, Yellow
            center_x_px, center_y_px = platform_center_pixels[0], platform_center_pixels[1]
            
            for i, motor_pos_px in enumerate(motor_positions_pixels):
                if len(motor_pos_px) == 2:
                    # Convert pixel position to meters relative to platform center
                    motor_x_px, motor_y_px = motor_pos_px[0], motor_pos_px[1]
                    motor_x_m = (motor_x_px - center_x_px) * pixel_to_meter_ratio
                    motor_y_m = (motor_y_px - center_y_px) * pixel_to_meter_ratio
                    
                    # Convert to canvas coordinates
                    mx, my = to_canvas(motor_x_m, motor_y_m)
                    
                    # Draw motor position
                    color = motor_colors[i % len(motor_colors)]
                    canvas.create_oval(mx - 5, my - 5, mx + 5, my + 5,
                                       fill=color, outline="", width=1)
                    canvas.create_oval(mx - 7, my - 7, mx + 7, my + 7,
                                       outline=color, width=2)
                    # Draw line from center to motor
                    canvas.create_line(center, center, mx, my, fill=color, width=1, dash=(2, 2))
                    # Label motor
                    canvas.create_text(mx + 12, my + 12, text=f"M{i+1}", 
                                      fill=color, font=("Arial", 10, "bold"))

if __name__ == "__main__":
    try:
        controller = StewartPlatformController()
        controller.run()
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()

