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
        Kp_x = self.config.get('pid', {}).get('Kp_x', 0.0)
        Ki_x = self.config.get('pid', {}).get('Ki_x', 0.0)
        Kd_x = self.config.get('pid', {}).get('Kd_x', 0.0)
        
        # Y-axis (pitch) PID gains
        Kp_y = self.config.get('pid', {}).get('Kp_y', 0.0)
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
        # Handle both "port" (singular) and "ports" (plural) for backward compatibility
        servo_config = self.config.get('servo', {})
        if 'port' in servo_config:
            self.servo_port = servo_config['port']
        elif 'ports' in servo_config and len(servo_config['ports']) > 0:
            # Use first port if "ports" array is provided
            self.servo_port = servo_config['ports'][0]
        else:
            self.servo_port = "COM3"  # Default fallback
        self.neutral_angles = servo_config.get('neutral_angles', [15, 15, 15])
        # Motor direction inversion: [False, False, False] means all motors normal direction
        # Set to True for any motor that spins the wrong way
        self.motor_direction_invert = self.config.get('servo', {}).get('motor_direction_invert', [False, False, False])
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
        
        # Thread-safe queue for most recent ball position measurement
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False    # Main run flag for clean shutdown
    
    def _get_default_config(self):
        """Return default configuration dictionary."""
        return {
            'camera': {
                'index': 0,
                'frame_width': 640,
                'frame_height': 480
            },
            'pid': {
                'Kp_x': 0.0,
                'Ki_x': 0.0,
                'Kd_x': 0.0,
                'Kp_y': 0.0,
                'Ki_y': 0.0,
                'Kd_y': 0.0
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
            self.servo_serial = serial.Serial(self.servo_port, 9600, timeout=1)
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
        
        # Store original PID output for debugging
        original_roll_angle = roll_angle
        
        # Apply roll direction inversion if configured (before IK or simplified mapping)
        roll_direction_invert = self.config.get('platform', {}).get('roll_direction_invert', False)
        if roll_direction_invert:
            roll_angle = -roll_angle
        
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
                motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(roll_angle, pitch_angle, original_roll_angle)
        else:
            # Use simplified trigonometric mapping (fallback)
            motor1_angle, motor2_angle, motor3_angle = self._simplified_motor_mapping(roll_angle, pitch_angle, original_roll_angle)
        
        # Clip to servo range (0-30 degrees as per Arduino code)
        motor1_angle = int(np.clip(motor1_angle, 0, 30))
        motor2_angle = int(np.clip(motor2_angle, 0, 30))
        motor3_angle = int(np.clip(motor3_angle, 0, 30))
        
        # DEBUG: Print motor angles being sent
        # print(f"[MOTOR] Roll={roll_angle:.1f}°, Pitch={pitch_angle:.1f}° -> "
        #       f"M1={motor1_angle}°, M2={motor2_angle}°, M3={motor3_angle}°")
        
        # Send all 3 servo commands to Arduino as 3 bytes
        if self.servo_serial:
            try:
                # CRITICAL FIX: Clear input buffer periodically to prevent Arduino debug messages from filling it up
                if self.servo_serial.in_waiting > 50:  # If buffer has accumulated data
                    self.servo_serial.reset_input_buffer()
                
                self.servo_serial.write(bytes([motor1_angle, motor2_angle, motor3_angle]))
                # Remove flush() - it can block and isn't necessary
                self.servo_serial.flush()  # COMMENT THIS OUT
            except Exception as e:
                print(f"[ARDUINO] Send failed: {e}")
        # Remove the else print statement (line 197) - it prints in tight loop
    
    def _simplified_motor_mapping(self, roll_angle, pitch_angle, original_roll_angle=None):
        """Simplified trigonometric mapping (fallback method).
        
        Args:
            roll_angle: Roll angle in degrees (after inversion if configured)
            pitch_angle: Pitch angle in degrees
            original_roll_angle: Original roll angle before inversion (for debugging)
            
        Returns:
            tuple: (motor1_angle, motor2_angle, motor3_angle) in degrees
        """
        if original_roll_angle is None:
            original_roll_angle = roll_angle
        # Motor positions in degrees (from +X axis, counter-clockwise)
        motor1_angle_deg = 90
        motor2_angle_deg = 210
        motor3_angle_deg = 330
        
        # Convert to radians
        motor1_angle_rad = np.radians(motor1_angle_deg)
        motor2_angle_rad = np.radians(motor2_angle_deg)
        motor3_angle_rad = np.radians(motor3_angle_deg)
        
        # Calculate height changes for each motor
        # Negative signs are needed for correct platform tilt direction
        # Note: roll_angle is already inverted in send_platform_tilt() if roll_direction_invert is True
        motor1_height = -roll_angle * np.cos(motor1_angle_rad) - pitch_angle * np.sin(motor1_angle_rad)
        motor2_height = -roll_angle * np.cos(motor2_angle_rad) - pitch_angle * np.sin(motor2_angle_rad)
        motor3_height = -roll_angle * np.cos(motor3_angle_rad) - pitch_angle * np.sin(motor3_angle_rad)
        
        # Convert height changes to motor angles
        # Scale factor: platform tilt degrees -> servo angle change
        # For most platforms, 1:1 mapping works, but you may need to adjust
        scale_factor = self.config.get('platform', {}).get('motor_scale_factor', 1.0)
        
        # Apply direction inversion if configured (multiply by -1 if inverted)
        motor1_dir = -1.0 if self.motor_direction_invert[0] else 1.0
        motor2_dir = -1.0 if self.motor_direction_invert[1] else 1.0
        motor3_dir = -1.0 if self.motor_direction_invert[2] else 1.0
        
        motor1_angle = self.neutral_angles[0] + motor1_height * scale_factor * motor1_dir
        motor2_angle = self.neutral_angles[1] + motor2_height * scale_factor * motor2_dir
        motor3_angle = self.neutral_angles[2] + motor3_height * scale_factor * motor3_dir
        
        # Get roll direction invert flag for debug output
        roll_direction_invert = self.config.get('platform', {}).get('roll_direction_invert', False)
        
        # DEBUG: Show intermediate calculations
        if abs(roll_angle) > 0.1 or abs(pitch_angle) > 0.1:
            invert_note = f" (inverted from {original_roll_angle:.1f}°)" if roll_direction_invert and abs(original_roll_angle - roll_angle) > 0.1 else ""
            print(f"[MAPPING] Roll={roll_angle:.1f}°{invert_note}, Pitch={pitch_angle:.1f}° -> "
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
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CONTROL] Error: {e}")
                break
        
        # Return to neutral on exit
        self.send_platform_tilt(0, 0)
        if self.servo_serial:
            self.servo_serial.close()
    
    def create_gui(self):
        """Build Tkinter GUI with large sliders and labeled controls."""
        self.root = tk.Tk()
        self.root.title("Stewart Platform PID Controller")
        self.root.geometry("650x800")
        
        # Title label
        ttk.Label(self.root, text="Stewart Platform Control", font=("Arial", 18, "bold")).pack(pady=10)
        
        # X-axis (Roll) controls
        ttk.Label(self.root, text="X-Axis (Roll) PID Gains", font=("Arial", 14, "bold")).pack(pady=5)
        
        # Kp_x slider with text box
        kp_x_frame = ttk.Frame(self.root)
        kp_x_frame.pack(pady=2)
        ttk.Label(kp_x_frame, text="Kp (Proportional)", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.kp_x_var = tk.DoubleVar(value=self.pid.Kp_x)
        kp_x_slider = ttk.Scale(kp_x_frame, from_=0, to=100, variable=self.kp_x_var,
                               orient=tk.HORIZONTAL, length=400)
        kp_x_slider.pack(side=tk.LEFT, padx=5)
        self.kp_x_entry = tk.Entry(kp_x_frame, width=8, font=("Arial", 10))
        self.kp_x_entry.insert(0, f"{self.pid.Kp_x:.3f}")
        self.kp_x_entry.pack(side=tk.LEFT, padx=5)
        self.kp_x_entry.bind('<Return>', lambda e: self.update_from_text_entry('kp_x', 0, 100))
        self.kp_x_entry.bind('<FocusOut>', lambda e: self.update_from_text_entry('kp_x', 0, 100))
        kp_x_slider.config(command=lambda v: self.update_from_slider('kp_x', v))
        
        # Ki_x slider with text box
        ki_x_frame = ttk.Frame(self.root)
        ki_x_frame.pack(pady=2)
        ttk.Label(ki_x_frame, text="Ki (Integral)", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.ki_x_var = tk.DoubleVar(value=self.pid.Ki_x)
        ki_x_slider = ttk.Scale(ki_x_frame, from_=0, to=10, variable=self.ki_x_var,
                               orient=tk.HORIZONTAL, length=400)
        ki_x_slider.pack(side=tk.LEFT, padx=5)
        self.ki_x_entry = tk.Entry(ki_x_frame, width=8, font=("Arial", 10))
        self.ki_x_entry.insert(0, f"{self.pid.Ki_x:.3f}")
        self.ki_x_entry.pack(side=tk.LEFT, padx=5)
        self.ki_x_entry.bind('<Return>', lambda e: self.update_from_text_entry('ki_x', 0, 10))
        self.ki_x_entry.bind('<FocusOut>', lambda e: self.update_from_text_entry('ki_x', 0, 10))
        ki_x_slider.config(command=lambda v: self.update_from_slider('ki_x', v))
        
        # Kd_x slider with text box
        kd_x_frame = ttk.Frame(self.root)
        kd_x_frame.pack(pady=2)
        ttk.Label(kd_x_frame, text="Kd (Derivative)", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.kd_x_var = tk.DoubleVar(value=self.pid.Kd_x)
        kd_x_slider = ttk.Scale(kd_x_frame, from_=0, to=20, variable=self.kd_x_var,
                               orient=tk.HORIZONTAL, length=400)
        kd_x_slider.pack(side=tk.LEFT, padx=5)
        self.kd_x_entry = tk.Entry(kd_x_frame, width=8, font=("Arial", 10))
        self.kd_x_entry.insert(0, f"{self.pid.Kd_x:.3f}")
        self.kd_x_entry.pack(side=tk.LEFT, padx=5)
        self.kd_x_entry.bind('<Return>', lambda e: self.update_from_text_entry('kd_x', 0, 20))
        self.kd_x_entry.bind('<FocusOut>', lambda e: self.update_from_text_entry('kd_x', 0, 20))
        kd_x_slider.config(command=lambda v: self.update_from_slider('kd_x', v))
        
        # Y-axis (Pitch) controls
        ttk.Label(self.root, text="Y-Axis (Pitch) PID Gains", font=("Arial", 14, "bold")).pack(pady=5)
        
        # Kp_y slider with text box
        kp_y_frame = ttk.Frame(self.root)
        kp_y_frame.pack(pady=2)
        ttk.Label(kp_y_frame, text="Kp (Proportional)", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.kp_y_var = tk.DoubleVar(value=self.pid.Kp_y)
        kp_y_slider = ttk.Scale(kp_y_frame, from_=0, to=100, variable=self.kp_y_var,
                               orient=tk.HORIZONTAL, length=400)
        kp_y_slider.pack(side=tk.LEFT, padx=5)
        self.kp_y_entry = tk.Entry(kp_y_frame, width=8, font=("Arial", 10))
        self.kp_y_entry.insert(0, f"{self.pid.Kp_y:.3f}")
        self.kp_y_entry.pack(side=tk.LEFT, padx=5)
        self.kp_y_entry.bind('<Return>', lambda e: self.update_from_text_entry('kp_y', 0, 100))
        self.kp_y_entry.bind('<FocusOut>', lambda e: self.update_from_text_entry('kp_y', 0, 100))
        kp_y_slider.config(command=lambda v: self.update_from_slider('kp_y', v))
        
        # Ki_y slider with text box
        ki_y_frame = ttk.Frame(self.root)
        ki_y_frame.pack(pady=2)
        ttk.Label(ki_y_frame, text="Ki (Integral)", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.ki_y_var = tk.DoubleVar(value=self.pid.Ki_y)
        ki_y_slider = ttk.Scale(ki_y_frame, from_=0, to=10, variable=self.ki_y_var,
                               orient=tk.HORIZONTAL, length=400)
        ki_y_slider.pack(side=tk.LEFT, padx=5)
        self.ki_y_entry = tk.Entry(ki_y_frame, width=8, font=("Arial", 10))
        self.ki_y_entry.insert(0, f"{self.pid.Ki_y:.3f}")
        self.ki_y_entry.pack(side=tk.LEFT, padx=5)
        self.ki_y_entry.bind('<Return>', lambda e: self.update_from_text_entry('ki_y', 0, 10))
        self.ki_y_entry.bind('<FocusOut>', lambda e: self.update_from_text_entry('ki_y', 0, 10))
        ki_y_slider.config(command=lambda v: self.update_from_slider('ki_y', v))
        
        # Kd_y slider with text box
        kd_y_frame = ttk.Frame(self.root)
        kd_y_frame.pack(pady=2)
        ttk.Label(kd_y_frame, text="Kd (Derivative)", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.kd_y_var = tk.DoubleVar(value=self.pid.Kd_y)
        kd_y_slider = ttk.Scale(kd_y_frame, from_=0, to=20, variable=self.kd_y_var,
                               orient=tk.HORIZONTAL, length=400)
        kd_y_slider.pack(side=tk.LEFT, padx=5)
        self.kd_y_entry = tk.Entry(kd_y_frame, width=8, font=("Arial", 10))
        self.kd_y_entry.insert(0, f"{self.pid.Kd_y:.3f}")
        self.kd_y_entry.pack(side=tk.LEFT, padx=5)
        self.kd_y_entry.bind('<Return>', lambda e: self.update_from_text_entry('kd_y', 0, 20))
        self.kd_y_entry.bind('<FocusOut>', lambda e: self.update_from_text_entry('kd_y', 0, 20))
        kd_y_slider.config(command=lambda v: self.update_from_slider('kd_y', v))
        
        # Setpoint controls
        ttk.Label(self.root, text="Setpoint", font=("Arial", 12)).pack(pady=5)
        
        # Setpoint X
        ttk.Label(self.root, text="Setpoint X (meters)", font=("Arial", 10)).pack()
        pos_range = 0.1  # Default range
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        setpoint_x_slider = ttk.Scale(self.root, from_=-pos_range, to=pos_range,
                                     variable=self.setpoint_x_var,
                                     orient=tk.HORIZONTAL, length=550)
        setpoint_x_slider.pack(pady=2)
        self.setpoint_x_label = ttk.Label(self.root, text=f"Setpoint X: {self.setpoint_x:.4f}m", font=("Arial", 9))
        self.setpoint_x_label.pack()
        
        # Setpoint Y
        ttk.Label(self.root, text="Setpoint Y (meters)", font=("Arial", 10)).pack()
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)
        setpoint_y_slider = ttk.Scale(self.root, from_=-pos_range, to=pos_range,
                                     variable=self.setpoint_y_var,
                                     orient=tk.HORIZONTAL, length=550)
        setpoint_y_slider.pack(pady=2)
        self.setpoint_y_label = ttk.Label(self.root, text=f"Setpoint Y: {self.setpoint_y:.4f}m", font=("Arial", 9))
        self.setpoint_y_label.pack()
        
        # Button group for actions
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)
        ttk.Button(button_frame, text="Reset Integrals",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=5)
        
        # Schedule periodic GUI update
        self.update_gui()
    
    def update_from_text_entry(self, param_name, min_val, max_val):
        """Update slider and PID controller from text entry box.
        
        Args:
            param_name: Name of parameter ('kp_x', 'ki_x', 'kd_x', 'kp_y', 'ki_y', 'kd_y')
            min_val: Minimum allowed value
            max_val: Maximum allowed value
        """
        try:
            # Get the entry widget and var for this parameter
            entry_widget = getattr(self, f"{param_name}_entry")
            var_widget = getattr(self, f"{param_name}_var")
            
            # Get value from text entry
            text_value = entry_widget.get().strip()
            if not text_value:
                # If empty, restore current value
                current_val = var_widget.get()
                entry_widget.delete(0, tk.END)
                entry_widget.insert(0, f"{current_val:.3f}")
                return
            
            value = float(text_value)
            # Clamp to valid range
            value = max(min_val, min(max_val, value))
            
            # Update the variable (which updates the slider)
            var_widget.set(value)
            
            # Update the text entry to show the clamped value
            entry_widget.delete(0, tk.END)
            entry_widget.insert(0, f"{value:.3f}")
            
        except ValueError:
            # Invalid input, restore current value
            var_widget = getattr(self, f"{param_name}_var")
            current_val = var_widget.get()
            entry_widget = getattr(self, f"{param_name}_entry")
            entry_widget.delete(0, tk.END)
            entry_widget.insert(0, f"{current_val:.3f}")
    
    def update_from_slider(self, param_name, value):
        """Update text entry box from slider.
        
        Args:
            param_name: Name of parameter ('kp_x', 'ki_x', 'kd_x', 'kp_y', 'ki_y', 'kd_y')
            value: New value from slider (as string)
        """
        try:
            entry_widget = getattr(self, f"{param_name}_entry")
            # Only update text entry if it doesn't have focus (user isn't typing)
            if self.root.focus_get() != entry_widget:
                # Update text entry with formatted value
                entry_widget.delete(0, tk.END)
                entry_widget.insert(0, f"{float(value):.3f}")
        except (ValueError, AttributeError):
            pass
    
    def update_gui(self):
        """Reflect latest values from sliders into program and update display."""
        if self.running:
            # PID parameters for X axis
            self.pid.set_gains_x(self.kp_x_var.get(), self.ki_x_var.get(), self.kd_x_var.get())
            # PID parameters for Y axis
            self.pid.set_gains_y(self.kp_y_var.get(), self.ki_y_var.get(), self.kd_y_var.get())
            
            # Setpoints
            self.setpoint_x = self.setpoint_x_var.get()
            self.setpoint_y = self.setpoint_y_var.get()
            self.pid.set_setpoint(self.setpoint_x, self.setpoint_y)
            
            # Update setpoint labels
            self.setpoint_x_label.config(text=f"Setpoint X: {self.setpoint_x:.4f}m")
            self.setpoint_y_label.config(text=f"Setpoint Y: {self.setpoint_y:.4f}m")
            
            # Call again after 50 ms
            self.root.after(50, self.update_gui)
    
    def reset_integral(self):
        """Clear integral error in PID (button handler)."""
        self.pid.reset_integral()
    
    def plot_results(self):
        """Show matplotlib plots of position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # X position trace
        axes[0, 0].plot(self.time_log, self.position_x_log, label="Ball X Position", linewidth=2)
        axes[0, 0].plot(self.time_log, self.setpoint_x_log, label="Setpoint X",
                       linestyle="--", linewidth=2)
        axes[0, 0].set_ylabel("Position X (m)")
        axes[0, 0].set_title("X-Axis (Roll) Control")
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Y position trace
        axes[0, 1].plot(self.time_log, self.position_y_log, label="Ball Y Position", linewidth=2, color='green')
        axes[0, 1].plot(self.time_log, self.setpoint_y_log, label="Setpoint Y",
                       linestyle="--", linewidth=2)
        axes[0, 1].set_ylabel("Position Y (m)")
        axes[0, 1].set_title("Y-Axis (Pitch) Control")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # X control output trace
        axes[1, 0].plot(self.time_log, self.control_x_log, label="Roll Output",
                       color="orange", linewidth=2)
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Roll Angle (degrees)")
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Y control output trace
        axes[1, 1].plot(self.time_log, self.control_y_log, label="Pitch Output",
                       color="red", linewidth=2)
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Pitch Angle (degrees)")
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
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

if __name__ == "__main__":
    try:
        controller = StewartPlatformController()
        controller.run()
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()

