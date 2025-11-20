# Stewart Platform Controller
# Simplified 2D ball balancer for Stewart platform with 3 motors
# Based on 1D beam balancer design

import cv2
import numpy as np
import json
import serial
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from threading import Thread
import queue
import logging
import os
from datetime import datetime
from collections import deque
from ball_detection_2d import BallDetector2D
from pid_controller_1d import PIDController1D

class StewartPlatformController:
    """Main controller for Stewart platform ball balancing system."""
    
    def __init__(self, config_file="config_stewart.json"):
        """Initialize controller, load config, set defaults and queues."""
        # Store config file path for logging
        self.config_file = config_file
        
        # Load config
        try:
            with open(config_file, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            error_msg = f"Config file {config_file} not found"
            print(f"[ERROR] {error_msg}")
            raise
        
        # Initialize 3 separate PID controllers (one for each motor)
        pid_config = self.config.get('pid', {})
        platform_config = self.config.get('platform', {})
        max_angle = platform_config.get('max_roll_angle', 15.0)
        
        # Motor PID controllers - each motor controls its own axis
        # CRITICAL FIX: Motor 1 can fall back to X gains (for backward compatibility)
        # Motors 2 and 3 default to 0 to prevent unwanted movement
        self.motor_pids = [
            PIDController1D(
                Kp=pid_config.get('Kp_m1', pid_config.get('Kp_x', 0.0)),  # Motor 1 can use Kp_x fallback
                Ki=pid_config.get('Ki_m1', pid_config.get('Ki_x', 0.0)),  # Motor 1 can use Ki_x fallback
                Kd=pid_config.get('Kd_m1', pid_config.get('Kd_x', 0.0)),  # Motor 1 can use Kd_x fallback
                output_limit=max_angle
            ),
            PIDController1D(
                Kp=pid_config.get('Kp_m2', 0.0),  # Motor 2 defaults to 0 (no fallback to Kp_y)
                Ki=pid_config.get('Ki_m2', 0.0),  # Motor 2 defaults to 0 (no fallback to Ki_y)
                Kd=pid_config.get('Kd_m2', 0.0),  # Motor 2 defaults to 0 (no fallback to Kd_y)
                output_limit=max_angle
            ),
            PIDController1D(
                Kp=pid_config.get('Kp_m3', 0.0),  # Motor 3 defaults to 0
                Ki=pid_config.get('Ki_m3', 0.0),  # Motor 3 defaults to 0
                Kd=pid_config.get('Kd_m3', 0.0),  # Motor 3 defaults to 0
                output_limit=max_angle
            )
        ]
        
        # Motor axis directions (in degrees from +X axis, counter-clockwise)
        # Motors at 120° intervals: 90°, 330°, 210° (motor 2 and 3 swapped)
        self.motor_angles_deg = [90, 330, 210]
        self.motor_angles_rad = [np.radians(a) for a in self.motor_angles_deg]
        
        # Initialize ball detector
        self.detector = BallDetector2D(config_file)
        
        # Servo configuration
        servo_config = self.config.get('servo', {})
        self.servo_port = servo_config.get('port', 'COM3')
        self.neutral_angles = servo_config.get('neutral_angles', [15, 15, 15])
        self.motor_direction_invert = servo_config.get('motor_direction_invert', [False, False, False])
        self.servo_serial = None
        
        # Debug flag for raw packet logging
        self.debug_packets = servo_config.get('debug_packets', False)
        
        # Logging configuration
        self.enable_logging = self.config.get('logging', {}).get('enable', True)
        self.log_dir = self.config.get('logging', {}).get('log_directory', 'logs')
        self.log_file = None
        self.logger = None
        
        # Initialize logging
        if self.enable_logging:
            self._setup_logging()
        
        # Controller state
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        
        # Data logs
        self.time_log = []
        self.position_x_log = []
        self.position_y_log = []
        self.setpoint_x_log = []
        self.setpoint_y_log = []
        self.control_m1_log = []
        self.control_m2_log = []
        self.control_m3_log = []
        self.start_time = None
        
        # Thread-safe queue for ball position
        self.position_queue = queue.Queue(maxsize=1)
        self.running = False
        self.angle_entry_widgets = []

        # Corruption warning throttling/tolerance
        platform_cfg = self.config.get('platform', {})
        self.corruption_warning_tolerance = platform_cfg.get('corruption_warning_tolerance_deg', 1.0)
        self.corruption_warning_interval = platform_cfg.get('corruption_warning_interval_s', 0.5)
        self._last_corruption_warning_time = 0.0

        # Live telemetry tracking
        self.latest_position_x = 0.0
        self.latest_position_y = 0.0
        self.latest_error_x = 0.0
        self.latest_error_y = 0.0
        self.live_plot_time = deque(maxlen=400)
        self.live_plot_setpoint_x = deque(maxlen=400)
        self.live_plot_setpoint_y = deque(maxlen=400)
        self.live_plot_position_x = deque(maxlen=400)
        self.live_plot_position_y = deque(maxlen=400)
        self.telemetry_window = None
        self.live_canvas = None
        self.live_ax_x = None
        self.live_ax_y = None
        self.live_line_setpoint_x = None
        self.live_line_position_x = None
        self.live_line_setpoint_y = None
        self.live_line_position_y = None
        self.live_error_label = None
    
    def _setup_logging(self):
        """Setup file logging for controller activity."""
        try:
            # Create logs directory if it doesn't exist
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
            
            # Create log filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = os.path.join(self.log_dir, f"controller_{timestamp}.log")
            self.log_file = log_filename
            
            # Setup logger
            self.logger = logging.getLogger('StewartPlatformController')
            self.logger.setLevel(logging.DEBUG)
            
            # Remove existing handlers
            self.logger.handlers = []
            
            # File handler - logs everything
            file_handler = logging.FileHandler(log_filename, mode='w', encoding='utf-8')
            file_handler.setLevel(logging.DEBUG)
            file_format = logging.Formatter('%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s', 
                                           datefmt='%Y-%m-%d %H:%M:%S')
            file_handler.setFormatter(file_format)
            self.logger.addHandler(file_handler)
            
            # Console handler - logs INFO and above
            console_handler = logging.StreamHandler()
            console_handler.setLevel(logging.INFO)
            console_format = logging.Formatter('[%(levelname)s] %(message)s')
            console_handler.setFormatter(console_format)
            self.logger.addHandler(console_handler)
            
            self.logger.info(f"Logging initialized - Log file: {log_filename}")
            self.logger.info(f"Controller configuration loaded from: {self.config_file}")
            
        except Exception as e:
            print(f"[WARNING] Failed to setup logging: {e}")
            self.enable_logging = False
            self.logger = None
    
    def _log(self, level, message, *args, **kwargs):
        """Helper method to log messages."""
        if self.logger:
            getattr(self.logger, level.lower())(message, *args, **kwargs)
        else:
            # Fallback to print if logging not available
            print(f"[{level}] {message}")
    
    def _update_motor_setpoints(self):
        """Update setpoints for each motor based on X/Y setpoint."""
        # Project setpoint onto each motor's axis
        # CRITICAL FIX: Only update setpoint for motors with non-zero gains
        # Motors with zero gains should keep setpoint = 0 to prevent any issues
        for i, (pid, angle_rad) in enumerate(zip(self.motor_pids, self.motor_angles_rad)):
            if pid.Kp == 0.0 and pid.Ki == 0.0 and pid.Kd == 0.0:
                # Motor has zero gains - keep setpoint at 0
                pid.set_setpoint(0.0)
            else:
                # Motor has active gains - update setpoint based on projection
                proj = self.setpoint_x * np.cos(angle_rad) + self.setpoint_y * np.sin(angle_rad)
                pid.set_setpoint(proj)
    
    def connect_servos(self):
        """Connect to Arduino servo controller."""
        try:
            self._log('info', f"Attempting to connect to Arduino on {self.servo_port}")
            self.servo_serial = serial.Serial(self.servo_port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            self.servo_serial.reset_input_buffer()
            self._log('info', f"Successfully connected to Arduino on {self.servo_port}")
            print(f"[ARDUINO] Connected to {self.servo_port}")
            return True
        except Exception as e:
            self._log('error', f"Failed to connect to Arduino: {e}")
            print(f"[ARDUINO] Failed to connect: {e}")
            self.servo_serial = None
            return False
    
    def send_motor_angles(self, motor_outputs):
        """Send motor control outputs directly to platform motors.
        
        Args:
            motor_outputs: List of 3 motor control outputs (degrees) from PID controllers
        """
        # Only send if serial connection is active and controller is running
        if not self.servo_serial or not self.running:
            return
        
        # Convert to servo angles with scale factor
        scale_factor = self.config.get('platform', {}).get('motor_scale_factor', 1.0)
        motor_angles = []
        for i, output in enumerate(motor_outputs):
            # CRITICAL FIX: If output is essentially zero, force to neutral immediately
            # This prevents any calculation that might result in non-neutral angles
            if abs(output) < 0.01:  # Essentially zero (accounting for floating point)
                motor_angles.append(15)  # Force to neutral
                continue
            
            # Get flip state from GUI checkbox if available, otherwise use config
            if hasattr(self, f'flip_m{i+1}_var'):
                is_flipped = getattr(self, f'flip_m{i+1}_var').get()
            else:
                is_flipped = self.motor_direction_invert[i]
            direction = -1.0 if is_flipped else 1.0
            angle = self.neutral_angles[i] + output * scale_factor * direction
            motor_angles.append(int(np.clip(angle, 0, 30)))
        
        # Send to Arduino
        try:
            # CRITICAL FIX: Aggressively clear input buffer BEFORE sending to prevent
            # Arduino from reading stale data mixed with new commands
            # Clear multiple times unconditionally (matches working test pattern)
            # Reduced delays for faster response
            for _ in range(2):  # Reduced from 3 to 2 iterations
                if self.servo_serial.in_waiting > 0:
                    self.servo_serial.reset_input_buffer()
                time.sleep(0.0005)  # Reduced delay (0.5ms instead of 1ms)
            
            # Send exactly 3 bytes atomically
            command_bytes = bytes(motor_angles)
            if len(command_bytes) != 3:
                print(f"[ARDUINO] ERROR: Invalid command length: {len(command_bytes)}")
                return
            
            # Calculate checksum (XOR of all bytes) for corruption detection
            checksum = command_bytes[0] ^ command_bytes[1] ^ command_bytes[2]
            
            # DEBUG: Show raw packet being sent
            packet_info = (f"PID Outputs: M1={motor_outputs[0]:.2f}°, M2={motor_outputs[1]:.2f}°, M3={motor_outputs[2]:.2f}° | "
                          f"Angles: M1={motor_angles[0]}°, M2={motor_angles[1]}°, M3={motor_angles[2]}° | "
                          f"Bytes: [{command_bytes[0]}, {command_bytes[1]}, {command_bytes[2]}] (hex: 0x{command_bytes.hex()}) | "
                          f"Checksum: {checksum}")
            
            if self.debug_packets:
                print(f"[PACKET] {packet_info}")
            
            self._log('debug', f"MOTOR_CMD: {packet_info}")
            
            # Double-check: Ensure any motor with zero output is at neutral
            # (This should already be handled above, but this is a safety check)
            for i, output in enumerate(motor_outputs):
                if abs(output) < 0.01 and motor_angles[i] != 15:
                    warning_msg = (f"CRITICAL: M{i+1} has zero PID output but angle is {motor_angles[i]}°! "
                                 f"Forcing to neutral. This should never happen.")
                    self._log('error', f"ARDUINO: {warning_msg}")
                    print(f"[ARDUINO] ERROR: {warning_msg}")
                    motor_angles[i] = 15  # Force to neutral
                    command_bytes = bytes(motor_angles)  # Recreate bytes with corrected angle
            
            # CRITICAL: Detect suspicious patterns before sending - check ALL motors
            corruption_warnings = []
            tolerance = self.corruption_warning_tolerance
            
            def outputs_differ(idx_a, idx_b):
                return abs(motor_outputs[idx_a] - motor_outputs[idx_b]) > tolerance
            
            # Pattern 1: Any motors identical when they shouldn't be
            if motor_angles[0] == motor_angles[1] and motor_angles[0] != 15:
                if motor_outputs[0] == 0.0 and motor_outputs[1] == 0.0:
                    corruption_warnings.append(f"M1=M2 ({motor_angles[0]}°) but should be neutral")
                elif outputs_differ(0, 1):
                    corruption_warnings.append(f"M1=M2 ({motor_angles[0]}°) but PID outputs differ: M1={motor_outputs[0]:.2f}°, M2={motor_outputs[1]:.2f}°")
            
            if motor_angles[0] == motor_angles[2] and motor_angles[0] != 15:
                if motor_outputs[0] == 0.0 and motor_outputs[2] == 0.0:
                    corruption_warnings.append(f"M1=M3 ({motor_angles[0]}°) but should be neutral")
                elif outputs_differ(0, 2):
                    corruption_warnings.append(f"M1=M3 ({motor_angles[0]}°) but PID outputs differ: M1={motor_outputs[0]:.2f}°, M3={motor_outputs[2]:.2f}°")
            
            if motor_angles[1] == motor_angles[2] and motor_angles[1] != 15:
                if motor_outputs[1] == 0.0 and motor_outputs[2] == 0.0:
                    corruption_warnings.append(f"M2=M3 ({motor_angles[1]}°) but should be neutral")
                elif outputs_differ(1, 2):
                    corruption_warnings.append(f"M2=M3 ({motor_angles[1]}°) but PID outputs differ: M2={motor_outputs[1]:.2f}°, M3={motor_outputs[2]:.2f}°")
            
            # Pattern 2: All motors identical (severe corruption)
            if motor_angles[0] == motor_angles[1] == motor_angles[2] and motor_angles[0] != 15:
                # Only flag if at least two outputs differ beyond tolerance
                if outputs_differ(0, 1) or outputs_differ(0, 2) or outputs_differ(1, 2):
                    corruption_warnings.append(f"ALL MOTORS IDENTICAL ({motor_angles[0]}°) - severe corruption!")
            
            # Pattern 3: Motor should be neutral but isn't (based on PID output)
            for i, (angle, output) in enumerate(zip(motor_angles, motor_outputs), 1):
                if abs(output) < 0.01 and angle != 15:
                    corruption_warnings.append(f"M{i} should be neutral (PID≈0.0) but angle is {angle}°")
            
            # Log warnings with throttling
            if corruption_warnings:
                now = time.time()
                if now - self._last_corruption_warning_time >= self.corruption_warning_interval:
                    for warning in corruption_warnings:
                        warning_msg = f"SUSPICIOUS PATTERN: {warning} | Possible byte corruption"
                        self._log('warning', f"ARDUINO: {warning_msg}")
                        print(f"[ARDUINO] WARNING: {warning_msg}")
                    self._last_corruption_warning_time = now
            
            bytes_written = self.servo_serial.write(command_bytes)
            if bytes_written != 3:
                warning_msg = f"Only {bytes_written} of 3 bytes sent"
                self._log('warning', f"ARDUINO: {warning_msg}")
                print(f"[ARDUINO] WARNING: {warning_msg}")
            
            # Flush to ensure data is sent immediately
            self.servo_serial.flush()
            
            # CRITICAL FIX: Read Arduino responses non-blocking to prevent buffer buildup
            # Since Arduino debug output is reduced (only every 100th command),
            # we rarely need to read. Make it truly non-blocking for speed.
            # This prevents debug messages from accumulating without adding delay
            try:
                # Only read if data is immediately available (no waiting)
                # With reduced debug output, 99% of commands won't have responses
                if self.servo_serial.in_waiting > 0:
                    # Read immediately available data only (non-blocking)
                    # Don't wait - if Arduino is still sending, we'll catch it next time
                    max_reads = 3  # Reduced limit - should be enough
                    read_count = 0
                    
                    while self.servo_serial.in_waiting > 0 and read_count < max_reads:
                        try:
                            # Non-blocking read - use timeout=0 or check available first
                            response = self.servo_serial.readline().decode('utf-8', errors='ignore').strip()
                            if response:
                                read_count += 1
                                # Log important responses (corruption, received bytes)
                                if "CORRUPTION" in response.upper() or "Received bytes:" in response or "Valid angles:" in response:
                                    self._log('debug', f"ARDUINO_RESPONSE: {response}")
                                    
                                    # Check if received bytes match sent bytes
                                    if "Received bytes:" in response:
                                        # Parse: "Received bytes: 15, 15, 15"
                                        parts = response.split(":")[1].strip().split(",")
                                        if len(parts) == 3:
                                            try:
                                                received = [int(p.strip()) for p in parts]
                                                if received != list(motor_angles):
                                                    error_msg = (f"CORRUPTION DETECTED! Sent: {motor_angles}, "
                                                                f"Arduino received: {received}")
                                                    self._log('error', f"ARDUINO: {error_msg}")
                                                    print(f"[ARDUINO] ERROR: {error_msg}")
                                            except ValueError:
                                                pass
                                # Even if not important, we read it to clear the buffer
                        except:
                            break
            except Exception as e:
                pass  # Ignore read errors, but we tried to clear the buffer
        except Exception as e:
            self._log('error', f"ARDUINO: Send failed - {e}")
            print(f"[ARDUINO] Send failed: {e}")
    
    def camera_thread(self):
        """Camera thread for ball detection."""
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
                self._log('debug', f"BALL_DETECT: Found at X={position_x_m:.4f}m, Y={position_y_m:.4f}m, radius={radius:.1f}px")
                try:
                    if self.position_queue.full():
                        self.position_queue.get_nowait()
                    self.position_queue.put_nowait((position_x_m, position_y_m))
                except Exception as e:
                    self._log('warning', f"BALL_DETECT: Queue error - {e}")
            else:
                self._log('debug', "BALL_DETECT: Not found")
            
            # Show video
            vis_frame, _, _, _ = self.detector.draw_detection(frame)
            cv2.imshow("Stewart Platform - Ball Tracking", vis_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC exits
                self.running = False
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    def control_thread(self):
        """Control thread running PID loop."""
        self._log('info', "Control thread started")
        
        if not self.connect_servos():
            self._log('warning', "No servos connected - running in simulation mode")
            print("[WARNING] No servos connected - running in simulation mode")
        
        self.start_time = time.time()
        self._log('info', f"Control loop started at {datetime.fromtimestamp(self.start_time).isoformat()}")
        
        # Set setpoints for each motor (project setpoint onto each motor's axis)
        self._update_motor_setpoints()
        self._log('info', f"Initial setpoints: X={self.setpoint_x:.4f}m, Y={self.setpoint_y:.4f}m")
        
        # Log PID gains and verify zero-gain motors
        for i, pid in enumerate(self.motor_pids):
            self._log('info', f"Motor {i+1} PID: Kp={pid.Kp:.3f}, Ki={pid.Ki:.3f}, Kd={pid.Kd:.3f}")
            if pid.Kp == 0.0 and pid.Ki == 0.0 and pid.Kd == 0.0:
                self._log('info', f"Motor {i+1}: All gains are zero - motor will be locked at neutral position")
                # Ensure setpoint is 0 and integral is reset
                pid.set_setpoint(0.0)
                pid.integral = 0.0
                pid.prev_error = 0.0
        
        # CRITICAL FIX: Send initial neutral position to ensure motors are at known state
        if self.servo_serial:
            self._log('info', "Sending initial neutral position to motors")
            self.send_motor_angles([0, 0, 0])
            time.sleep(0.1)  # Give Arduino time to process
        
        # Control loop rate: 30 Hz = ~33ms per cycle for responsive control
        CONTROL_LOOP_RATE_HZ = 30
        CONTROL_LOOP_DT = 1.0 / CONTROL_LOOP_RATE_HZ  # ~0.033 seconds
        
        # Track last motor outputs to send hold commands when ball not detected
        last_motor_outputs = [0.0, 0.0, 0.0]
        last_ball_detected_time = None
        BALL_TIMEOUT_SECONDS = 0.5  # If no ball for 0.5s, return to neutral
        
        while self.running:
            loop_start_time = time.time()
            try:
                # Get latest ball position (with short timeout for responsiveness)
                # Use get_nowait first, then short timeout fallback
                try:
                    position_x, position_y = self.position_queue.get_nowait()
                except queue.Empty:
                    # If no data, wait briefly but don't block the whole loop
                    try:
                        position_x, position_y = self.position_queue.get(timeout=0.01)  # Reduced from 0.1s
                    except queue.Empty:
                        # No ball data - send hold command or return to neutral
                        current_time = time.time()
                        
                        # Check if we should return to neutral (no ball for too long)
                        if last_ball_detected_time is None:
                            last_ball_detected_time = current_time
                        
                        time_since_last_ball = current_time - last_ball_detected_time
                        
                        if time_since_last_ball > BALL_TIMEOUT_SECONDS:
                            # No ball for too long - return to neutral
                            hold_outputs = [0.0, 0.0, 0.0]
                            self._log('debug', f"CONTROL: No ball for {time_since_last_ball:.2f}s, returning to neutral")
                        else:
                            # Send last known command to hold position
                            hold_outputs = last_motor_outputs.copy()
                            self._log('debug', f"CONTROL: No ball data, holding last position (M1={hold_outputs[0]:.2f}°, M2={hold_outputs[1]:.2f}°, M3={hold_outputs[2]:.2f}°)")
                        
                        # Send hold/neutral command to keep motors responsive
                        self.send_motor_angles(hold_outputs)
                        
                        # Sleep to maintain control loop rate
                        elapsed = time.time() - loop_start_time
                        sleep_time = max(0, CONTROL_LOOP_DT - elapsed)
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                        continue
                
                # Project ball position onto each motor's axis
                # Each motor responds to position component along its direction
                motor_positions = []
                for angle_rad in self.motor_angles_rad:
                    # Project position onto motor axis (cos for x, sin for y)
                    proj = position_x * np.cos(angle_rad) + position_y * np.sin(angle_rad)
                    motor_positions.append(proj)
                
                # Compute PID output for each motor
                motor_outputs = []
                motor_errors = []
                for i, (pid, pos) in enumerate(zip(self.motor_pids, motor_positions)):
                    # CRITICAL FIX: If motor has all gains = 0, force output to 0.0
                    # This prevents any movement even if PID state somehow gets corrupted
                    if pid.Kp == 0.0 and pid.Ki == 0.0 and pid.Kd == 0.0:
                        output = 0.0
                        # Reset integral to prevent any accumulation
                        pid.integral = 0.0
                        # Don't even call update() to avoid any state changes
                    else:
                        output = pid.update(pos)
                    motor_outputs.append(output)
                    # Calculate error for logging
                    error = pid.setpoint - pos
                    motor_errors.append(error)
                    
                    # Log if a motor with zero gains is somehow producing non-zero output
                    if pid.Kp == 0.0 and pid.Ki == 0.0 and pid.Kd == 0.0 and abs(output) > 0.001:
                        self._log('warning', f"MOTOR {i+1}: Has zero gains but output={output:.3f}° - forcing to 0.0")
                        motor_outputs[i] = 0.0
                        output = 0.0
                
                # Log PID calculations
                self._log('debug', (f"PID_CALC: Errors M1={motor_errors[0]:.4f}, M2={motor_errors[1]:.4f}, M3={motor_errors[2]:.4f} | "
                                   f"Outputs M1={motor_outputs[0]:.3f}°, M2={motor_outputs[1]:.3f}°, M3={motor_outputs[2]:.3f}°"))
                
                # Update last known outputs and ball detection time
                last_motor_outputs = motor_outputs.copy()
                last_ball_detected_time = time.time()

                # Update live telemetry for GUI plotting
                self.latest_position_x = position_x
                self.latest_position_y = position_y
                self.latest_error_x = self.setpoint_x - position_x
                self.latest_error_y = self.setpoint_y - position_y
                
                # Send commands with valid ball detection
                self.send_motor_angles(motor_outputs)
                
                # Log data
                current_time = time.time() - self.start_time
                self.time_log.append(current_time)
                self.position_x_log.append(position_x)
                self.position_y_log.append(position_y)
                self.setpoint_x_log.append(self.setpoint_x)
                self.setpoint_y_log.append(self.setpoint_y)
                self.control_m1_log.append(motor_outputs[0])
                self.control_m2_log.append(motor_outputs[1])
                self.control_m3_log.append(motor_outputs[2])
                
                # Log control cycle summary
                self._log('info', (f"CONTROL: t={current_time:.3f}s | "
                                  f"Pos X={position_x:.4f}m, Y={position_y:.4f}m | "
                                  f"Setpoint X={self.setpoint_x:.4f}m, Y={self.setpoint_y:.4f}m | "
                                  f"Motor Outputs M1={motor_outputs[0]:.2f}°, M2={motor_outputs[1]:.2f}°, M3={motor_outputs[2]:.2f}°"))
                
                print(f"Pos: X={position_x:.3f}m, Y={position_y:.3f}m | "
                      f"Motor Outputs: M1={motor_outputs[0]:.1f}°, M2={motor_outputs[1]:.1f}°, M3={motor_outputs[2]:.1f}°")
                
                # Maintain control loop rate for consistent timing
                elapsed = time.time() - loop_start_time
                sleep_time = max(0, CONTROL_LOOP_DT - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                self._log('error', f"CONTROL: Error in control loop - {e}")
                print(f"[CONTROL] Error: {e}")
                # Sleep even on error to prevent tight loop, then continue
                elapsed = time.time() - loop_start_time
                sleep_time = max(0, CONTROL_LOOP_DT - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue  # Continue instead of break to keep running
        
        # Return to neutral on exit
        self._log('info', "Control thread stopping - returning motors to neutral")
        if self.servo_serial:
            self.send_motor_angles([0, 0, 0])
            time.sleep(0.1)  # Give Arduino time to process
            self.servo_serial.close()
            self._log('info', "Arduino connection closed")
    
    def _create_motor_angle_inputs(self, parent):
        """Create simple text inputs for motor angles."""
        angle_frame = ttk.LabelFrame(parent, text="Motor Angles (degrees)")
        angle_frame.pack(pady=(5, 5), padx=10, fill=tk.X)
        
        self.angle_entry_widgets = []
        
        for i, angle_deg in enumerate(self.motor_angles_deg):
            row = ttk.Frame(angle_frame)
            row.pack(pady=2, fill=tk.X)
            ttk.Label(row, text=f"Motor {i+1}", width=10, font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=4)
            angle_var = tk.StringVar(value=f"{angle_deg:.1f}")
            angle_entry = ttk.Entry(row, textvariable=angle_var, width=8, font=("Arial", 9))
            angle_entry.pack(side=tk.LEFT, padx=4)
            ttk.Label(row, text="°").pack(side=tk.LEFT)
            angle_entry.bind('<Return>', lambda e, idx=i: self._update_motor_angle_from_entry(idx))
            angle_entry.bind('<FocusOut>', lambda e, idx=i: self._update_motor_angle_from_entry(idx))
            
            self.angle_entry_widgets.append({
                'var': angle_var,
                'entry': angle_entry,
                'index': i
            })
    
    def _update_motor_angle_from_entry(self, motor_index):
        """Update motor angle from text entry field."""
        try:
            entry_data = self.angle_entry_widgets[motor_index]
            angle_str = entry_data['var'].get().strip()
            
            if not angle_str:
                # Restore current value if empty
                entry_data['var'].set(f"{self.motor_angles_deg[motor_index]:.1f}")
                return
            
            # Parse angle value
            angle_deg = float(angle_str)
            
            # Normalize to 0-360 range
            angle_deg = angle_deg % 360
            if angle_deg < 0:
                angle_deg += 360
            
            # Update motor angle
            self.motor_angles_deg[motor_index] = angle_deg
            self.motor_angles_rad[motor_index] = np.radians(angle_deg)
            
            # Update entry field with normalized value
            entry_data['var'].set(f"{angle_deg:.1f}")
            
            # Update motor control labels
            self._update_motor_control_labels()
            
            # Log the change
            motor_num = motor_index + 1
            self._log('info', f"MOTOR_POS: Motor {motor_num} position set to {angle_deg:.1f}° (from entry)")
            print(f"[MOTOR] Motor {motor_num} position set to: {angle_deg:.1f}°")
            
        except ValueError:
            # Invalid input - restore current value
            entry_data = self.angle_entry_widgets[motor_index]
            entry_data['var'].set(f"{self.motor_angles_deg[motor_index]:.1f}")
            print(f"[MOTOR] Invalid angle value, restored to {self.motor_angles_deg[motor_index]:.1f}°")
    
    def _update_motor_control_labels(self):
        """Update motor control section labels with new angles."""
        # Update motor labels in the GUI
        for i, (angle_deg, label_widget) in enumerate(zip(self.motor_angles_deg, self.motor_label_widgets)):
            motor_num = i + 1
            new_label = f"M{motor_num} ({angle_deg:.0f}°)"
            label_widget.config(text=new_label)
    
    def _create_motor_pid_controls(self, parent, motor_num, motor_angle_deg):
        """Create PID controls for a single motor.
        
        Returns:
            The label widget for the motor (so it can be updated)
        """
        motor_label = f"M{motor_num} ({motor_angle_deg:.0f}°)"
        label_widget = ttk.Label(parent, text=motor_label, 
                 font=("Arial", 10, "bold"))
        label_widget.pack(pady=(3, 2))
        
        pid = self.motor_pids[motor_num - 1]
        
        # Kp
        kp_frame = ttk.Frame(parent)
        kp_frame.pack(pady=1, fill=tk.X)
        ttk.Label(kp_frame, text="Kp", font=("Arial", 9), width=8).pack(side=tk.LEFT, padx=2)
        kp_var = tk.DoubleVar(value=pid.Kp)
        kp_slider = ttk.Scale(kp_frame, from_=0, to=20, variable=kp_var,
                             orient=tk.HORIZONTAL, length=300)
        kp_slider.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        kp_entry = tk.Entry(kp_frame, width=8, font=("Arial", 9))
        kp_entry.insert(0, f"{pid.Kp:.3f}")
        kp_entry.pack(side=tk.LEFT, padx=2)
        kp_entry.bind('<Return>', lambda e, m=motor_num: self.update_from_text_entry(f'kp_m{m}', 0, 20))
        kp_entry.bind('<FocusOut>', lambda e, m=motor_num: self.update_from_text_entry(f'kp_m{m}', 0, 20))
        kp_slider.config(command=lambda v, m=motor_num: self.update_from_slider(f'kp_m{m}', v))
        setattr(self, f'kp_m{motor_num}_var', kp_var)
        setattr(self, f'kp_m{motor_num}_entry', kp_entry)
        
        # Ki
        ki_frame = ttk.Frame(parent)
        ki_frame.pack(pady=1, fill=tk.X)
        ttk.Label(ki_frame, text="Ki", font=("Arial", 9), width=8).pack(side=tk.LEFT, padx=2)
        ki_var = tk.DoubleVar(value=pid.Ki)
        ki_slider = ttk.Scale(ki_frame, from_=0, to=5, variable=ki_var,
                             orient=tk.HORIZONTAL, length=300)
        ki_slider.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ki_entry = tk.Entry(ki_frame, width=8, font=("Arial", 9))
        ki_entry.insert(0, f"{pid.Ki:.3f}")
        ki_entry.pack(side=tk.LEFT, padx=2)
        ki_entry.bind('<Return>', lambda e, m=motor_num: self.update_from_text_entry(f'ki_m{m}', 0, 5))
        ki_entry.bind('<FocusOut>', lambda e, m=motor_num: self.update_from_text_entry(f'ki_m{m}', 0, 5))
        ki_slider.config(command=lambda v, m=motor_num: self.update_from_slider(f'ki_m{m}', v))
        setattr(self, f'ki_m{motor_num}_var', ki_var)
        setattr(self, f'ki_m{motor_num}_entry', ki_entry)
        
        # Kd
        kd_frame = ttk.Frame(parent)
        kd_frame.pack(pady=1, fill=tk.X)
        ttk.Label(kd_frame, text="Kd", font=("Arial", 9), width=8).pack(side=tk.LEFT, padx=2)
        kd_var = tk.DoubleVar(value=pid.Kd)
        kd_slider = ttk.Scale(kd_frame, from_=0, to=10, variable=kd_var,
                             orient=tk.HORIZONTAL, length=300)
        kd_slider.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        kd_entry = tk.Entry(kd_frame, width=8, font=("Arial", 9))
        kd_entry.insert(0, f"{pid.Kd:.3f}")
        kd_entry.pack(side=tk.LEFT, padx=2)
        kd_entry.bind('<Return>', lambda e, m=motor_num: self.update_from_text_entry(f'kd_m{m}', 0, 10))
        kd_entry.bind('<FocusOut>', lambda e, m=motor_num: self.update_from_text_entry(f'kd_m{m}', 0, 10))
        kd_slider.config(command=lambda v, m=motor_num: self.update_from_slider(f'kd_m{m}', v))
        setattr(self, f'kd_m{motor_num}_var', kd_var)
        setattr(self, f'kd_m{motor_num}_entry', kd_entry)
        
        # Flip motor direction checkbox
        flip_frame = ttk.Frame(parent)
        flip_frame.pack(pady=1, fill=tk.X)
        flip_var = tk.BooleanVar(value=self.motor_direction_invert[motor_num - 1])
        flip_checkbox = ttk.Checkbutton(flip_frame, text="Flip", 
                                       variable=flip_var)
        flip_checkbox.pack(side=tk.LEFT, padx=2)
        setattr(self, f'flip_m{motor_num}_var', flip_var)
        
        return label_widget
    
    def _create_master_pid_controls(self, parent):
        """Create master sliders that tune all motor PID gains together."""
        ttk.Label(parent, text="Master PID (All Motors)", font=("Arial", 10, "bold")).pack(pady=(10, 2))
        master_frame = ttk.Frame(parent)
        master_frame.pack(pady=(0, 8), fill=tk.X, padx=5)
        
        # Helper to build each master slider row
        def build_master_row(name, from_, to_, var, command):
            row = ttk.Frame(master_frame)
            row.pack(pady=2, fill=tk.X)
            ttk.Label(row, text=name, font=("Arial", 9), width=10).pack(side=tk.LEFT, padx=2)
            slider = ttk.Scale(row, from_=from_, to=to_, orient=tk.HORIZONTAL, length=280,
                               variable=var, command=command)
            slider.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)
            value_label = ttk.Label(row, text=f"{var.get():.3f}", width=8, font=("Arial", 9))
            value_label.pack(side=tk.LEFT, padx=2)
            return slider, value_label
        
        # Determine initial values (use motor 1 as reference)
        pid_ref = self.motor_pids[0]
        self.master_kp_var = tk.DoubleVar(value=pid_ref.Kp)
        self.master_ki_var = tk.DoubleVar(value=pid_ref.Ki)
        self.master_kd_var = tk.DoubleVar(value=pid_ref.Kd)
        
        def on_master_change(param, value, label_widget):
            try:
                value = float(value)
            except ValueError:
                return
            label_widget.config(text=f"{value:.3f}")
            for motor_idx in range(1, 4):
                var_widget = getattr(self, f"{param}_m{motor_idx}_var")
                var_widget.set(value)
                self.update_from_slider(f"{param}_m{motor_idx}", value)
        
        self.master_kp_slider, self.master_kp_label = build_master_row(
            "Master Kp", 0, 20, self.master_kp_var,
            lambda v, lbl=None: on_master_change('kp', v, self.master_kp_label))
        self.master_ki_slider, self.master_ki_label = build_master_row(
            "Master Ki", 0, 5, self.master_ki_var,
            lambda v, lbl=None: on_master_change('ki', v, self.master_ki_label))
        self.master_kd_slider, self.master_kd_label = build_master_row(
            "Master Kd", 0, 10, self.master_kd_var,
            lambda v, lbl=None: on_master_change('kd', v, self.master_kd_label))
    
    def _build_telemetry_window_contents(self, parent):
        """Build contents of the telemetry window."""
        panel = ttk.Frame(parent)
        panel.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)
        
        figure = Figure(figsize=(5.5, 3.5), dpi=100)
        self.live_ax_x = figure.add_subplot(211)
        self.live_ax_y = figure.add_subplot(212, sharex=self.live_ax_x)
        self.live_ax_x.set_ylabel("X (m)")
        self.live_ax_y.set_ylabel("Y (m)")
        self.live_ax_y.set_xlabel("Time (s)")
        self.live_ax_x.grid(True, alpha=0.3)
        self.live_ax_y.grid(True, alpha=0.3)
        self.live_line_setpoint_x, = self.live_ax_x.plot([], [], label="Setpoint X", color="tab:orange")
        self.live_line_position_x, = self.live_ax_x.plot([], [], label="Ball X", color="tab:blue")
        self.live_line_setpoint_y, = self.live_ax_y.plot([], [], label="Setpoint Y", color="tab:red")
        self.live_line_position_y, = self.live_ax_y.plot([], [], label="Ball Y", color="tab:green")
        self.live_ax_x.legend(loc="upper right")
        self.live_ax_y.legend(loc="upper right")
        
        self.live_canvas = FigureCanvasTkAgg(figure, master=panel)
        self.live_canvas.draw()
        self.live_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.live_error_label = ttk.Label(panel, text="Error X=0.0000m | Y=0.0000m", font=("Arial", 9, "bold"))
        self.live_error_label.pack(pady=4)
    
    def open_telemetry_window(self):
        """Open (or focus) telemetry window for live plotting."""
        if self.telemetry_window and self.telemetry_window.winfo_exists():
            self.telemetry_window.lift()
            return
        
        self.telemetry_window = tk.Toplevel(self.root)
        self.telemetry_window.title("Live Telemetry")
        self.telemetry_window.geometry("700x600")
        self.telemetry_window.protocol("WM_DELETE_WINDOW", self._close_telemetry_window)
        
        ttk.Label(self.telemetry_window, text="Real-Time Setpoint vs Position",
                  font=("Arial", 11, "bold")).pack(pady=4)
        self._build_telemetry_window_contents(self.telemetry_window)
        self._update_live_plot()
    
    def _close_telemetry_window(self):
        """Handle telemetry window closing."""
        if self.telemetry_window:
            try:
                self.telemetry_window.destroy()
            except Exception:
                pass
        self.telemetry_window = None
        self.live_canvas = None
        self.live_ax_x = None
        self.live_ax_y = None
        self.live_line_setpoint_x = None
        self.live_line_position_x = None
        self.live_line_setpoint_y = None
        self.live_line_position_y = None
        self.live_error_label = None

    def _update_live_plot(self):
        """Append latest telemetry and refresh live plot display."""
        if not self.live_canvas or self.start_time is None:
            return
        
        current_time = time.time() - self.start_time
        self.live_plot_time.append(current_time)
        self.live_plot_setpoint_x.append(self.setpoint_x)
        self.live_plot_setpoint_y.append(self.setpoint_y)
        self.live_plot_position_x.append(self.latest_position_x)
        self.live_plot_position_y.append(self.latest_position_y)
        
        self.live_line_setpoint_x.set_data(self.live_plot_time, self.live_plot_setpoint_x)
        self.live_line_position_x.set_data(self.live_plot_time, self.live_plot_position_x)
        self.live_line_setpoint_y.set_data(self.live_plot_time, self.live_plot_setpoint_y)
        self.live_line_position_y.set_data(self.live_plot_time, self.live_plot_position_y)
        
        # Limit X-axis to recent window (e.g., last 15 seconds)
        window = 15
        if self.live_plot_time:
            t_max = self.live_plot_time[-1]
            t_min = max(0, t_max - window)
            self.live_ax_x.set_xlim(t_min, t_max if t_max > t_min else t_min + 1)
        
        self.live_ax_x.relim()
        self.live_ax_x.autoscale_view(scalex=False, scaley=True)
        self.live_ax_y.relim()
        self.live_ax_y.autoscale_view(scalex=False, scaley=True)
        
        # Update error label
        error_x = self.latest_error_x
        error_y = self.latest_error_y
        if self.live_error_label:
            self.live_error_label.config(text=f"Error X={error_x:.4f}m | Y={error_y:.4f}m")
        
        self.live_canvas.draw_idle()
    
    def create_gui(self):
        """Build GUI with motor-based controls."""
        self.root = tk.Tk()
        self.root.title("Stewart Platform PID Controller")
        self.root.geometry("550x700")
        
        # Title
        ttk.Label(self.root, text="Stewart Platform Control", 
                 font=("Arial", 12, "bold")).pack(pady=3)
        
        # Motor angle inputs
        self._create_motor_angle_inputs(self.root)
        
        # Store motor label widgets for live updates
        self.motor_label_widgets = []
        
        # Create controls for each motor
        for i, angle_deg in enumerate(self.motor_angles_deg):
            label_widget = self._create_motor_pid_controls(self.root, i+1, angle_deg)
            self.motor_label_widgets.append(label_widget)
        
        # Master PID controls
        self._create_master_pid_controls(self.root)
        
        # Setpoint
        ttk.Label(self.root, text="Setpoint", font=("Arial", 10, "bold")).pack(pady=(5, 2))
        calib = self.config.get('calibration', {})
        pos_min_x = calib.get('position_min_x_m', -0.1)
        pos_max_x = calib.get('position_max_x_m', 0.1)
        pos_min_y = calib.get('position_min_y_m', -0.1)
        pos_max_y = calib.get('position_max_y_m', 0.1)
        
        # Setpoint X
        ttk.Label(self.root, text="Setpoint X (m)", font=("Arial", 9)).pack()
        self.setpoint_x_var = tk.DoubleVar(value=self.setpoint_x)
        setpoint_x_slider = ttk.Scale(self.root, from_=pos_min_x, to=pos_max_x,
                                     variable=self.setpoint_x_var,
                                     orient=tk.HORIZONTAL, length=400)
        setpoint_x_slider.pack(pady=2)
        self.setpoint_x_label = ttk.Label(self.root, text=f"X: {self.setpoint_x:.4f}m", 
                                         font=("Arial", 9))
        self.setpoint_x_label.pack()
        
        # Setpoint Y
        ttk.Label(self.root, text="Setpoint Y (m)", font=("Arial", 9)).pack()
        self.setpoint_y_var = tk.DoubleVar(value=self.setpoint_y)
        setpoint_y_slider = ttk.Scale(self.root, from_=pos_min_y, to=pos_max_y,
                                     variable=self.setpoint_y_var,
                                     orient=tk.HORIZONTAL, length=400)
        setpoint_y_slider.pack(pady=2)
        self.setpoint_y_label = ttk.Label(self.root, text=f"Y: {self.setpoint_y:.4f}m", 
                                         font=("Arial", 9))
        self.setpoint_y_label.pack()
        
        # Buttons
        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=5)
        ttk.Button(button_frame, text="Reset Integral",
                   command=self.reset_integral).pack(side=tk.LEFT, padx=3)
        ttk.Button(button_frame, text="Plot Results",
                   command=self.plot_results).pack(side=tk.LEFT, padx=3)
        ttk.Button(button_frame, text="Stop",
                   command=self.stop).pack(side=tk.LEFT, padx=3)
        ttk.Button(button_frame, text="Open Telemetry",
                   command=self.open_telemetry_window).pack(side=tk.LEFT, padx=3)
        
        # Schedule GUI updates
        self.update_gui()
    
    def update_from_text_entry(self, param_name, min_val, max_val):
        """Update slider and PID controller from text entry box.
        
        Args:
            param_name: Name of parameter ('kp_m1', 'ki_m1', 'kd_m1', etc.)
            min_val: Minimum allowed value
            max_val: Maximum allowed value
        """
        try:
            entry_widget = getattr(self, f"{param_name}_entry")
            var_widget = getattr(self, f"{param_name}_var")
            
            text_value = entry_widget.get().strip()
            if not text_value:
                current_val = var_widget.get()
                entry_widget.delete(0, tk.END)
                entry_widget.insert(0, f"{current_val:.3f}")
                return
            
            value = float(text_value)
            value = max(min_val, min(max_val, value))
            
            var_widget.set(value)
            entry_widget.delete(0, tk.END)
            entry_widget.insert(0, f"{value:.3f}")
            
            # Extract motor number and parameter type
            if param_name.startswith('kp_m'):
                motor_num = int(param_name[4]) - 1
                old_value = self.motor_pids[motor_num].Kp
                self.motor_pids[motor_num].Kp = value
                self._log('info', f"PID_GAIN: Motor {motor_num+1} Kp changed from {old_value:.3f} to {value:.3f}")
            elif param_name.startswith('ki_m'):
                motor_num = int(param_name[4]) - 1
                old_value = self.motor_pids[motor_num].Ki
                self.motor_pids[motor_num].Ki = value
                self._log('info', f"PID_GAIN: Motor {motor_num+1} Ki changed from {old_value:.3f} to {value:.3f}")
            elif param_name.startswith('kd_m'):
                motor_num = int(param_name[4]) - 1
                old_value = self.motor_pids[motor_num].Kd
                self.motor_pids[motor_num].Kd = value
                self._log('info', f"PID_GAIN: Motor {motor_num+1} Kd changed from {old_value:.3f} to {value:.3f}")
            
        except (ValueError, AttributeError):
            var_widget = getattr(self, f"{param_name}_var")
            current_val = var_widget.get()
            entry_widget = getattr(self, f"{param_name}_entry")
            entry_widget.delete(0, tk.END)
            entry_widget.insert(0, f"{current_val:.3f}")
    
    def update_from_slider(self, param_name, value):
        """Update text entry box from slider.
        
        Args:
            param_name: Name of parameter ('kp_m1', 'ki_m1', 'kd_m1', etc.)
            value: New value from slider (as string)
        """
        try:
            entry_widget = getattr(self, f"{param_name}_entry")
            if self.root.focus_get() != entry_widget:
                entry_widget.delete(0, tk.END)
                entry_widget.insert(0, f"{float(value):.3f}")
            
            # Update PID controller
            if param_name.startswith('kp_m'):
                motor_num = int(param_name[4]) - 1
                old_value = self.motor_pids[motor_num].Kp
                new_value = float(value)
                self.motor_pids[motor_num].Kp = new_value
                if abs(old_value - new_value) > 0.001:  # Only log if significant change
                    self._log('info', f"PID_GAIN: Motor {motor_num+1} Kp changed from {old_value:.3f} to {new_value:.3f}")
            elif param_name.startswith('ki_m'):
                motor_num = int(param_name[4]) - 1
                old_value = self.motor_pids[motor_num].Ki
                new_value = float(value)
                self.motor_pids[motor_num].Ki = new_value
                if abs(old_value - new_value) > 0.001:  # Only log if significant change
                    self._log('info', f"PID_GAIN: Motor {motor_num+1} Ki changed from {old_value:.3f} to {new_value:.3f}")
            elif param_name.startswith('kd_m'):
                motor_num = int(param_name[4]) - 1
                old_value = self.motor_pids[motor_num].Kd
                new_value = float(value)
                self.motor_pids[motor_num].Kd = new_value
                if abs(old_value - new_value) > 0.001:  # Only log if significant change
                    self._log('info', f"PID_GAIN: Motor {motor_num+1} Kd changed from {old_value:.3f} to {new_value:.3f}")
        except (ValueError, AttributeError):
            pass
    
    def update_gui(self):
        """Update GUI and PID parameters."""
        if self.running:
            # Update setpoints
            old_setpoint_x = self.setpoint_x
            old_setpoint_y = self.setpoint_y
            self.setpoint_x = self.setpoint_x_var.get()
            self.setpoint_y = self.setpoint_y_var.get()
            
            # Log setpoint changes
            if abs(old_setpoint_x - self.setpoint_x) > 0.0001 or abs(old_setpoint_y - self.setpoint_y) > 0.0001:
                self._log('info', f"SETPOINT: Changed to X={self.setpoint_x:.4f}m, Y={self.setpoint_y:.4f}m")
            
            self._update_motor_setpoints()
            
            # Update setpoint labels
            self.setpoint_x_label.config(text=f"Setpoint X: {self.setpoint_x:.4f}m")
            self.setpoint_y_label.config(text=f"Setpoint Y: {self.setpoint_y:.4f}m")

            # Refresh live plot
            self._update_live_plot()
            
            # Schedule next update
            self.root.after(50, self.update_gui)
    
    def reset_integral(self):
        """Reset PID integral terms for all motors."""
        for pid in self.motor_pids:
            pid.reset_integral()
        self._log('info', "PID integral terms reset for all motors")
        print("[RESET] Integral terms reset for all motors")
    
    def plot_results(self):
        """Plot position and control logs."""
        if not self.time_log:
            print("[PLOT] No data to plot")
            return
        
        # Get platform radius from config
        platform_radius = self.config.get('platform_radius_m', 0.1)
        
        # Create figure with 2D trajectory plot and time series
        from matplotlib.gridspec import GridSpec
        fig = plt.figure(figsize=(16, 10))
        gs = GridSpec(2, 3, figure=fig, hspace=0.3, wspace=0.3)
        
        # 2D trajectory plot (left side, spans 2 rows)
        ax_2d = fig.add_subplot(gs[:, 0])  # All rows, first column
        ax_2d.plot(self.position_x_log, self.position_y_log, 'b-', linewidth=2, 
                  label='Ball Trajectory', alpha=0.7)
        ax_2d.scatter(self.position_x_log[0], self.position_y_log[0], 
                     color='green', s=100, marker='o', label='Start', zorder=5)
        ax_2d.scatter(self.position_x_log[-1], self.position_y_log[-1], 
                     color='red', s=100, marker='s', label='End', zorder=5)
        ax_2d.scatter(self.setpoint_x, self.setpoint_y, 
                     color='orange', s=150, marker='*', label='Setpoint', zorder=5)
        
        # Draw platform outline (circle)
        circle = plt.Circle((0, 0), platform_radius, fill=False, 
                           color='black', linewidth=2, linestyle='--', label='Platform')
        ax_2d.add_patch(circle)
        
        ax_2d.set_xlabel('X Position (m)', fontsize=11)
        ax_2d.set_ylabel('Y Position (m)', fontsize=11)
        ax_2d.set_title('2D Ball Trajectory on Platform', fontsize=12, fontweight='bold')
        ax_2d.set_aspect('equal', adjustable='box')
        ax_2d.grid(True, alpha=0.3)
        ax_2d.legend(loc='upper right')
        ax_2d.axhline(y=0, color='k', linewidth=0.5, alpha=0.3)
        ax_2d.axvline(x=0, color='k', linewidth=0.5, alpha=0.3)
        
        # X position time series
        ax_x = fig.add_subplot(gs[0, 1])
        ax_x.plot(self.time_log, self.position_x_log, label="Ball X Position", linewidth=2)
        ax_x.plot(self.time_log, self.setpoint_x_log, label="Setpoint X",
                 linestyle="--", linewidth=2)
        ax_x.set_ylabel("Position X (m)")
        ax_x.set_title("X Position")
        ax_x.legend()
        ax_x.grid(True, alpha=0.3)
        
        # Y position time series
        ax_y = fig.add_subplot(gs[0, 2])
        ax_y.plot(self.time_log, self.position_y_log, label="Ball Y Position", 
                 linewidth=2, color='green')
        ax_y.plot(self.time_log, self.setpoint_y_log, label="Setpoint Y",
                 linestyle="--", linewidth=2)
        ax_y.set_ylabel("Position Y (m)")
        ax_y.set_title("Y Position")
        ax_y.legend()
        ax_y.grid(True, alpha=0.3)
        
        # Motor 1 control output
        ax_ctrl_m1 = fig.add_subplot(gs[1, 1])
        ax_ctrl_m1.plot(self.time_log, self.control_m1_log, label="Motor 1 Output",
                       color="orange", linewidth=2)
        ax_ctrl_m1.set_xlabel("Time (s)")
        ax_ctrl_m1.set_ylabel("Motor 1 Angle (degrees)")
        ax_ctrl_m1.legend()
        ax_ctrl_m1.grid(True, alpha=0.3)
        
        # Motor 2 control output
        ax_ctrl_m2 = fig.add_subplot(gs[1, 2])
        ax_ctrl_m2.plot(self.time_log, self.control_m2_log, label="Motor 2 Output",
                       color="red", linewidth=2)
        ax_ctrl_m2.set_xlabel("Time (s)")
        ax_ctrl_m2.set_ylabel("Motor 2 Angle (degrees)")
        ax_ctrl_m2.legend()
        ax_ctrl_m2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def stop(self):
        """Stop controller and clean up."""
        self.running = False
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass
    
    def run(self):
        """Start controller."""
        self._log('info', "="*60)
        self._log('info', "Stewart Platform Controller Starting")
        self._log('info', "="*60)
        print("[INFO] Starting Stewart Platform Controller")
        print("Use sliders to tune PID gains in real-time")
        print("Close camera window or click Stop to exit")
        if self.enable_logging and self.log_file:
            print(f"[INFO] Logging to: {self.log_file}")
        self.running = True
        
        # Start threads
        cam_thread = Thread(target=self.camera_thread, daemon=True)
        ctrl_thread = Thread(target=self.control_thread, daemon=True)
        cam_thread.start()
        ctrl_thread.start()
        
        # Run GUI
        self.create_gui()
        self.root.mainloop()
        
        # Cleanup
        self.running = False
        self._log('info', "="*60)
        self._log('info', "Stewart Platform Controller Stopped")
        if self.time_log:
            duration = self.time_log[-1] if self.time_log else 0
            self._log('info', f"Total runtime: {duration:.2f} seconds")
            self._log('info', f"Total control cycles: {len(self.time_log)}")
        self._log('info', "="*60)
        print("[INFO] Controller stopped")
        if self.enable_logging and self.log_file:
            print(f"[INFO] Log saved to: {self.log_file}")

if __name__ == "__main__":
    try:
        controller = StewartPlatformController()
        controller.run()
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
