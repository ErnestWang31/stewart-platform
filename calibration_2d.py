# 2D Calibration System for Stewart Platform Ball Control
# Interactive calibration tool for color detection, geometry, and platform limits
# Generates config_stewart.json file for use with Stewart platform controller

import cv2
import numpy as np
import json
import math
import serial
import time
from datetime import datetime

class StewartPlatformCalibrator:
    """Interactive calibration system for Stewart platform ball control setup."""
    
    def __init__(self):
        """Initialize calibration parameters and default values."""
        # Physical system parameters
        self.PLATFORM_RADIUS_M = 0.15  # Known platform radius in meters (circular platform)
        self.PLATFORM_DIAMETER_M = 2 * self.PLATFORM_RADIUS_M  # Platform diameter
        
        # Camera configuration
        self.CAM_INDEX = 1  # Default camera index
        self.FRAME_W, self.FRAME_H = 640, 480  # Frame dimensions
        
        # Calibration state tracking
        self.current_frame = None  # Current video frame
        self.phase = "color"  # Current phase: "color", "geometry", "complete"
        
        # Color calibration data
        self.hsv_samples = []  # Collected HSV color samples
        self.lower_hsv = None  # Lower HSV bound for ball detection
        self.upper_hsv = None  # Upper HSV bound for ball detection
        
        # Geometry calibration data (circular platform)
        self.platform_center = None  # Platform center pixel coordinates
        self.platform_edge_point = None  # Platform edge point for radius calculation (deprecated, kept for compatibility)
        self.motor_attachment_points = []  # List of 2 motor attachment points clicked in order
        self.platform_radius_pixels = None  # Platform radius in pixels
        self.pixel_to_meter_ratio = None  # Conversion ratio from pixels to meters (uniform for circle)
        self.motor_positions_pixels = [None, None, None]  # Pixel positions of motors 1, 2, 3
        self.motor_angles_deg = [None, None, None]  # Angles of motors 1, 2, 3 in degrees (from +X axis, counter-clockwise)
        self.axis_rotation_deg = 0.0  # Rotation angle to align camera frame with platform coordinate system
        
        # Platform hardware configuration
        # All 3 motors are controlled through a single Arduino on COM3
        # The Arduino uses an I2C PWM Servo Driver to control all 3 servos
        self.servo_serial = None  # Single serial connection to Arduino
        self.servo_port = "COM3"  # Single port for all motors
        self.neutral_angles = [15, 15, 15]  # Servo neutral position angles
        self.debug_packets = False  # Debug flag for raw packet logging
        
        # Load existing config to preserve settings like roll_direction_invert
        self.load_existing_config()
        
        # Position limits (simple defaults based on platform radius)
        self.position_min_x = -self.PLATFORM_RADIUS_M
        self.position_max_x = self.PLATFORM_RADIUS_M
        self.position_min_y = -self.PLATFORM_RADIUS_M
        self.position_max_y = self.PLATFORM_RADIUS_M
    
    def load_existing_config(self):
        """Load existing config file to preserve settings like servo ports and neutral angles."""
        try:
            with open("config_stewart.json", "r") as f:
                config = json.load(f)
            
            # Load servo settings if they exist
            if "servo" in config:
                servo_config = config["servo"]
                # Handle both "port" (singular) and "ports" (plural) for backward compatibility
                if "port" in servo_config:
                    self.servo_port = servo_config["port"]
                elif "ports" in servo_config and len(servo_config["ports"]) > 0:
                    # Use first port if "ports" array is provided
                    self.servo_port = servo_config["ports"][0]
                if "neutral_angles" in servo_config:
                    self.neutral_angles = servo_config["neutral_angles"]
            
            # Load camera settings if they exist
            if "camera" in config:
                cam_config = config["camera"]
                if "index" in cam_config:
                    self.CAM_INDEX = cam_config["index"]
                if "frame_width" in cam_config:
                    self.FRAME_W = cam_config["frame_width"]
                if "frame_height" in cam_config:
                    self.FRAME_H = cam_config["frame_height"]
            
            # Load platform radius if it exists
            if "platform_radius_m" in config:
                self.PLATFORM_RADIUS_M = config["platform_radius_m"]
                self.PLATFORM_DIAMETER_M = 2 * self.PLATFORM_RADIUS_M
                
        except FileNotFoundError:
            # Config file doesn't exist yet, use defaults
            pass
        except Exception as e:
            print(f"[WARNING] Error loading existing config: {e}, using defaults")
    
    def connect_servos(self):
        """Establish serial connection to Arduino controlling all 3 servos.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.servo_serial = serial.Serial(self.servo_port, 9600, timeout=1)
            time.sleep(2)  # Allow time for Arduino to initialize
            # Flush any initial data
            self.servo_serial.reset_input_buffer()
            print(f"[ARDUINO] Connected to {self.servo_port}")
            return True
        except Exception as e:
            print(f"[ARDUINO] Failed to connect to {self.servo_port}: {e}")
            print(f"[ARDUINO] Limits will be estimated without servos")
            self.servo_serial = None
            return False
    
    def send_servo_angles(self, angles):
        """Send angle commands to all 3 servo motors via single Arduino.
        
        All 3 motors are controlled through a single Arduino with I2C PWM Servo Driver.
        Sends all 3 angles as 3 bytes: [motor1_angle, motor2_angle, motor3_angle]
        
        Args:
            angles: List of 3 angles in degrees (0-30 range)
        """
        if not self.servo_serial:
            return
        
        try:
            # Clip all angles to safe range (0-30 degrees)
            motor1_angle = int(np.clip(angles[0], 0, 30))
            motor2_angle = int(np.clip(angles[1], 0, 30))
            motor3_angle = int(np.clip(angles[2], 0, 30))
            
            # CRITICAL FIX: Aggressively clear input buffer BEFORE sending to prevent
            # Arduino from reading stale data mixed with new commands
            # Clear multiple times unconditionally (matches working test pattern)
            for _ in range(3):
                if self.servo_serial.in_waiting > 0:
                    self.servo_serial.reset_input_buffer()
                time.sleep(0.001)  # Tiny delay between clears
            
            # Send all 3 servo commands to Arduino as 3 bytes atomically
            command_bytes = bytes([motor1_angle, motor2_angle, motor3_angle])
            if len(command_bytes) != 3:
                print(f"[ARDUINO] ERROR: Invalid command length: {len(command_bytes)}")
                return
            
            # DEBUG: Show raw packet being sent
            if self.debug_packets:
                print(f"[PACKET] Input Angles: M1={angles[0]:.2f}°, M2={angles[1]:.2f}°, M3={angles[2]:.2f}°")
                print(f"[PACKET] Clipped Angles: M1={motor1_angle}°, M2={motor2_angle}°, M3={motor3_angle}°")
                print(f"[PACKET] Raw Bytes (decimal): [{command_bytes[0]}, {command_bytes[1]}, {command_bytes[2]}]")
                print(f"[PACKET] Raw Bytes (hex): 0x{command_bytes.hex()}")
                print(f"[PACKET] Raw Bytes (binary): {bin(command_bytes[0])}, {bin(command_bytes[1])}, {bin(command_bytes[2])}")
            
            bytes_written = self.servo_serial.write(command_bytes)
            if bytes_written != 3:
                print(f"[ARDUINO] WARNING: Only {bytes_written} of 3 bytes sent")
            
            # Flush to ensure data is sent immediately
            self.servo_serial.flush()
            
            # CRITICAL FIX: Always read Arduino responses to prevent buffer buildup
            # This prevents debug messages from accumulating and causing byte misalignment
            time.sleep(0.01)  # Small delay for Arduino to respond
            try:
                if self.servo_serial.in_waiting > 0:
                    # Read and discard Arduino debug messages
                    self.servo_serial.readline().decode('utf-8', errors='ignore')
            except Exception:
                pass  # Ignore read errors, but we tried to clear the buffer
        except Exception as e:
            print(f"[ARDUINO] Send failed: {e}")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse click events for interactive calibration.
        
        Args:
            event: OpenCV mouse event type
            x, y: Mouse click coordinates
            flags: Additional event flags
            param: User data (unused)
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.phase == "color":
                # Color sampling phase - collect HSV samples at click point
                self.sample_color(x, y)
            elif self.phase == "geometry":
                if len(self.motor_attachment_points) == 0:
                    # First click: first motor attachment point (Motor 1)
                    self.motor_attachment_points.append((x, y))
                    print(f"[GEO] Motor 1 attachment point selected at ({x}, {y})")
                    print("[GEO] Now click on the second motor attachment point (Motor 2)")
                elif len(self.motor_attachment_points) == 1:
                    # Second click: second motor attachment point (Motor 2)
                    self.motor_attachment_points.append((x, y))
                    print(f"[GEO] Motor 2 attachment point selected at ({x}, {y})")
                    print("[GEO] Now click on the third motor attachment point (Motor 3)")
                elif len(self.motor_attachment_points) == 2:
                    # Third click: third motor attachment point (Motor 3)
                    self.motor_attachment_points.append((x, y))
                    print(f"[GEO] Motor 3 attachment point selected at ({x}, {y})")
                    self.calculate_geometry_from_motors()
    
    def sample_color(self, x, y):
        """Sample HSV color values in a 5x5 region around click point.
        
        Args:
            x, y: Center coordinates for color sampling
        """
        if self.current_frame is None:
            return
        
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        
        # Sample 5x5 region around click point
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                px, py = x + dx, y + dy
                # Check bounds and collect valid samples
                if 0 <= px < hsv.shape[1] and 0 <= py < hsv.shape[0]:
                    self.hsv_samples.append(hsv[py, px])
        
        # Update HSV bounds based on collected samples
        if self.hsv_samples:
            samples = np.array(self.hsv_samples)
            
            # Calculate adaptive margins for each HSV channel
            h_margin = max(5, (np.max(samples[:, 0]) - np.min(samples[:, 0])) * 0.1)
            s_margin = max(10, (np.max(samples[:, 1]) - np.min(samples[:, 1])) * 0.15)
            v_margin = max(10, (np.max(samples[:, 2]) - np.min(samples[:, 2])) * 0.15)
            
            # Set lower bounds with margin
            self.lower_hsv = [
                max(0, np.min(samples[:, 0]) - h_margin),
                max(0, np.min(samples[:, 1]) - s_margin),
                max(0, np.min(samples[:, 2]) - v_margin)
            ]
            
            # Set upper bounds with margin
            self.upper_hsv = [
                min(179, np.max(samples[:, 0]) + h_margin),
                min(255, np.max(samples[:, 1]) + s_margin),
                min(255, np.max(samples[:, 2]) + v_margin)
            ]
            
            print(f"[COLOR] Samples: {len(self.hsv_samples)}")
    
    def calculate_geometry_from_motors(self):
        """Calculate platform geometry from 3 motor attachment points.
        
        Motors are arranged at 120° intervals on a circle. Given 3 motor points
        clicked in order (Motor 1, Motor 2, Motor 3), we can determine:
        - Platform center (circumcenter of the triangle)
        - Platform radius (average distance from center to motors)
        - Motor positions and angles
        - Axis orientation
        """
        if len(self.motor_attachment_points) < 3:
            return
        
        # Get all three motor attachment points
        p1 = np.array(self.motor_attachment_points[0])  # Motor 1
        p2 = np.array(self.motor_attachment_points[1])  # Motor 2
        p3 = np.array(self.motor_attachment_points[2])  # Motor 3
        
        # Calculate the circumcenter of the triangle formed by the 3 points
        # This is the center of the circle passing through all 3 points
        # Method: Find intersection of perpendicular bisectors of two sides
        
        # Perpendicular bisector of side p1-p2
        mid12 = (p1 + p2) / 2.0
        dir12 = p2 - p1
        perp12 = np.array([-dir12[1], dir12[0]])  # Perpendicular vector
        
        # Perpendicular bisector of side p2-p3
        mid23 = (p2 + p3) / 2.0
        dir23 = p3 - p2
        perp23 = np.array([-dir23[1], dir23[0]])  # Perpendicular vector
        
        # Find intersection of the two bisectors
        # Solve: mid12 + t * perp12 = mid23 + s * perp23
        # Rearranged: t * perp12 - s * perp23 = mid23 - mid12
        
        # Use cross product method to solve
        diff = mid23 - mid12
        denom = perp12[0] * perp23[1] - perp12[1] * perp23[0]
        
        if abs(denom) < 1e-6:
            # Points are collinear, fall back to average position
            print("[WARNING] Motor points appear collinear, using average as center")
            self.platform_center = tuple(((p1 + p2 + p3) / 3.0).astype(int))
        else:
            t = (diff[0] * perp23[1] - diff[1] * perp23[0]) / denom
            center = mid12 + t * perp12
            self.platform_center = tuple(center.astype(int))
        
        center_array = np.array(self.platform_center)
        
        # Calculate radius as average distance from center to each motor
        dist1 = np.linalg.norm(p1 - center_array)
        dist2 = np.linalg.norm(p2 - center_array)
        dist3 = np.linalg.norm(p3 - center_array)
        self.platform_radius_pixels = (dist1 + dist2 + dist3) / 3.0
        
        # Calculate angles for each motor from center
        def angle_from_center(center, point):
            vec = point - center
            return np.degrees(np.arctan2(vec[1], vec[0]))
        
        angle1 = angle_from_center(center_array, p1)
        angle2 = angle_from_center(center_array, p2)
        angle3 = angle_from_center(center_array, p3)
        
        # Normalize angles to [0, 360)
        angle1 = angle1 % 360
        angle2 = angle2 % 360
        angle3 = angle3 % 360
        
        # Store motor positions and angles
        self.motor_positions_pixels[0] = tuple(p1.astype(int))
        self.motor_positions_pixels[1] = tuple(p2.astype(int))
        self.motor_positions_pixels[2] = tuple(p3.astype(int))
        self.motor_angles_deg[0] = angle1
        self.motor_angles_deg[1] = angle2
        self.motor_angles_deg[2] = angle3
        
        # Verify 120° spacing (with some tolerance)
        angles_sorted = sorted([angle1, angle2, angle3])
        # Calculate differences (wrapping around 360)
        diff1 = (angles_sorted[1] - angles_sorted[0]) % 360
        diff2 = (angles_sorted[2] - angles_sorted[1]) % 360
        diff3 = (angles_sorted[0] - angles_sorted[2] + 360) % 360
        
        # Check if differences are close to 120° (within 30° tolerance)
        spacing_errors = [abs(d - 120) for d in [diff1, diff2, diff3]]
        max_error = max(spacing_errors)
        if max_error > 30:
            print(f"[WARNING] Motor spacing may not be exactly 120° (max error: {max_error:.1f}°)")
        else:
            print(f"[GEO] Motor spacing verified: {diff1:.1f}°, {diff2:.1f}°, {diff3:.1f}°")
        
        # Determine axis orientation
        # Motor 1 is at 90° in the standard configuration (top, +Y axis)
        # Standard: Motor 1 at 90°, Motor 2 at 210°, Motor 3 at 330°
        # We want to rotate so that Motor 1 aligns with 90°
        standard_motor1_angle = 90.0
        rotation_needed = standard_motor1_angle - angle1
        self.axis_rotation_deg = rotation_needed
        
        # Calculate pixel-to-meter conversion ratio
        # Use the calculated radius in pixels and the configured radius in meters
        self.pixel_to_meter_ratio = self.PLATFORM_RADIUS_M / self.platform_radius_pixels
        
        # Store for compatibility with ball detection (X and Y use same ratio for circle)
        self.pixel_to_meter_ratio_x = self.pixel_to_meter_ratio
        self.pixel_to_meter_ratio_y = self.pixel_to_meter_ratio
        
        # Also store edge point for backward compatibility (use Motor 1 as reference)
        self.platform_edge_point = self.motor_positions_pixels[0]
        
        print(f"[GEO] Platform center: {self.platform_center}")
        print(f"[GEO] Platform radius: {self.platform_radius_pixels:.2f} pixels = {self.PLATFORM_RADIUS_M:.4f} meters")
        print(f"[GEO] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
        print(f"[GEO] Motor 1: {self.motor_positions_pixels[0]} at {self.motor_angles_deg[0]:.1f}°")
        print(f"[GEO] Motor 2: {self.motor_positions_pixels[1]} at {self.motor_angles_deg[1]:.1f}°")
        print(f"[GEO] Motor 3: {self.motor_positions_pixels[2]} at {self.motor_angles_deg[2]:.1f}°")
        print(f"[GEO] Axis rotation: {self.axis_rotation_deg:.1f}°")
        
        # Advance to complete phase
        self.phase = "complete"
    
    def calculate_geometry(self):
        """Legacy method for backward compatibility. Redirects to new method."""
        if len(self.motor_attachment_points) >= 3:
            self.calculate_geometry_from_motors()
        elif len(self.motor_attachment_points) >= 2:
            # Fallback: if only 2 points, use old 2-point method
            self.calculate_geometry_from_motors()
        elif self.platform_center is not None and self.platform_edge_point is not None:
            # Old 2-point method (center + edge)
            dx = self.platform_edge_point[0] - self.platform_center[0]
            dy = self.platform_edge_point[1] - self.platform_center[1]
            self.platform_radius_pixels = np.sqrt(dx*dx + dy*dy)
            self.pixel_to_meter_ratio = self.PLATFORM_RADIUS_M / self.platform_radius_pixels
            self.pixel_to_meter_ratio_x = self.pixel_to_meter_ratio
            self.pixel_to_meter_ratio_y = self.pixel_to_meter_ratio
            print(f"[GEO] Platform center: {self.platform_center}")
            print(f"[GEO] Platform radius: {self.platform_radius_pixels:.2f} pixels = {self.PLATFORM_RADIUS_M:.4f} meters")
            print(f"[GEO] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
            self.phase = "complete"
    
    def detect_ball_position(self, frame):
        """Detect ball in frame and return position in meters from center.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            tuple or None: (x, y) ball position in meters from center, None if not detected
        """
        if not self.lower_hsv:
            return None
        
        # Convert to HSV and create color mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.lower_hsv, dtype=np.uint8)
        upper = np.array(self.upper_hsv, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        
        # Clean up mask with morphological operations
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours in mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # Get largest contour (assumed to be ball)
        largest = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest)
        
        # Filter out very small detections
        if radius < 5:
            return None
        
        # Convert pixel position to meters from platform center
        # Use calibrated platform center if available, otherwise use frame center
        if self.platform_center:
            center_x, center_y = self.platform_center
        else:
            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2
        
        pixel_offset_x = x - center_x
        pixel_offset_y = y - center_y
        
        if self.pixel_to_meter_ratio:
            meters_offset_x = pixel_offset_x * self.pixel_to_meter_ratio
            meters_offset_y = pixel_offset_y * self.pixel_to_meter_ratio
            return (meters_offset_x, meters_offset_y)
        else:
            return None
    
    def save_config(self):
        """Save all calibration results to config_stewart.json file.
        
        Preserves existing config fields (like motor_direction_invert) and only updates
        calibration-related fields.
        """
        # Load existing config if it exists, otherwise start with empty dict
        # Handle both file not found and JSON decode errors (corrupted file)
        try:
            with open("config_stewart.json", "r") as f:
                config = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            if isinstance(e, json.JSONDecodeError):
                print(f"[WARNING] Config file is corrupted, starting with fresh config: {e}")
            config = {}
        
        # Update timestamp
        config["timestamp"] = datetime.now().isoformat()
        
        # Update platform info
        config["platform_type"] = "circular"
        config["platform_radius_m"] = float(self.PLATFORM_RADIUS_M)
        # Convert platform_center to native Python types
        if self.platform_center:
            config["platform_center_pixels"] = [int(x) for x in self.platform_center]
        else:
            config["platform_center_pixels"] = None
        config["platform_radius_pixels"] = float(self.platform_radius_pixels) if self.platform_radius_pixels else None
        
        # Store motor positions and angles if available
        if all(self.motor_positions_pixels):
            config["motor_positions_pixels"] = [[int(x) for x in pos] for pos in self.motor_positions_pixels]
            config["motor_angles_deg"] = [float(angle) for angle in self.motor_angles_deg]
            config["axis_rotation_deg"] = float(self.axis_rotation_deg)
        
        # Update camera settings
        if "camera" not in config:
            config["camera"] = {}
        config["camera"]["index"] = int(self.CAM_INDEX)
        config["camera"]["frame_width"] = int(self.FRAME_W)
        config["camera"]["frame_height"] = int(self.FRAME_H)
        
        # Update ball detection settings
        if "ball_detection" not in config:
            config["ball_detection"] = {}
        config["ball_detection"]["lower_hsv"] = [float(x) for x in self.lower_hsv] if self.lower_hsv else None
        config["ball_detection"]["upper_hsv"] = [float(x) for x in self.upper_hsv] if self.upper_hsv else None
        
        # Update calibration data
        if "calibration" not in config:
            config["calibration"] = {}
        config["calibration"]["pixel_to_meter_ratio"] = float(self.pixel_to_meter_ratio) if self.pixel_to_meter_ratio else None
        config["calibration"]["pixel_to_meter_ratio_x"] = float(self.pixel_to_meter_ratio) if self.pixel_to_meter_ratio else None
        config["calibration"]["pixel_to_meter_ratio_y"] = float(self.pixel_to_meter_ratio) if self.pixel_to_meter_ratio else None
        # Set position limits based on platform radius
        config["calibration"]["position_min_x_m"] = float(-self.PLATFORM_RADIUS_M)
        config["calibration"]["position_max_x_m"] = float(self.PLATFORM_RADIUS_M)
        config["calibration"]["position_min_y_m"] = float(-self.PLATFORM_RADIUS_M)
        config["calibration"]["position_max_y_m"] = float(self.PLATFORM_RADIUS_M)
        
        # Update servo settings (preserve existing fields like motor_direction_invert)
        if "servo" not in config:
            config["servo"] = {}
        # Update calibration-related servo fields
        # Store as "port" (singular) since all motors use one Arduino
        config["servo"]["port"] = str(self.servo_port)
        # Also keep "ports" for backward compatibility (array with single port)
        config["servo"]["ports"] = [str(self.servo_port)]
        config["servo"]["neutral_angles"] = [int(a) for a in self.neutral_angles]
        # IMPORTANT: motor_direction_invert and all other existing servo fields are 
        # automatically preserved since we load the existing config first and only 
        # update specific fields above
        
        # Set default PID values only if they don't exist
        if "pid" not in config:
            config["pid"] = {}
        if "Kp_x" not in config["pid"]:
            config["pid"]["Kp_x"] = 10.0
        if "Ki_x" not in config["pid"]:
            config["pid"]["Ki_x"] = 0.0
        if "Kd_x" not in config["pid"]:
            config["pid"]["Kd_x"] = 0.0
        if "Kp_y" not in config["pid"]:
            config["pid"]["Kp_y"] = 10.0
        if "Ki_y" not in config["pid"]:
            config["pid"]["Ki_y"] = 0.0
        if "Kd_y" not in config["pid"]:
            config["pid"]["Kd_y"] = 0.0
        
        # Set default platform limits only if they don't exist
        # IMPORTANT: Preserve all existing platform fields (roll_direction_invert, use_inverse_kinematics, etc.)
        if "platform" not in config:
            config["platform"] = {}
        else:
            # Explicitly preserve all existing platform fields (don't overwrite them)
            # This ensures roll_direction_invert and other settings are kept
            pass  # All existing fields in config["platform"] are already preserved
        
        # Only set defaults if fields don't exist
        if "max_roll_angle" not in config["platform"]:
            config["platform"]["max_roll_angle"] = 15.0
        if "max_pitch_angle" not in config["platform"]:
            config["platform"]["max_pitch_angle"] = 15.0
        # Note: roll_direction_invert, use_inverse_kinematics, motor_scale_factor, 
        # and all other existing platform fields are automatically preserved since
        # we load the existing config first and only add defaults for missing fields
        
        # Write configuration to JSON file
        with open("config_stewart.json", "w") as f:
            json.dump(config, f, indent=2)
        print("[SAVE] Configuration saved to config_stewart.json (existing fields preserved)")
    
    
    def draw_overlay(self, frame):
        """Draw calibration status and instructions overlay on frame.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            numpy.ndarray: Frame with overlay graphics and text
        """
        overlay = frame.copy()
        
        # Phase-specific instruction text
        phase_text = {
            "color": "Click on ball to sample colors. Press 'c' when done.",
            "geometry": "Click on 3 motor attachment points in order (Motor 1, Motor 2, Motor 3)",
            "complete": "Calibration complete! Press 's' to save"
        }
        
        # Draw current phase and instructions
        cv2.putText(overlay, f"Phase: {self.phase}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, phase_text.get(self.phase, ""), (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show color calibration progress
        if self.hsv_samples:
            cv2.putText(overlay, f"Color samples: {len(self.hsv_samples)}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show HSV color range after calibration
        if self.lower_hsv and self.upper_hsv:
            y_offset = 120
            cv2.putText(overlay, "HSV Color Range:", (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(overlay, f"Lower: [{self.lower_hsv[0]:.1f}, {self.lower_hsv[1]:.1f}, {self.lower_hsv[2]:.1f}]",
                       (10, y_offset + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(overlay, f"Upper: [{self.upper_hsv[0]:.1f}, {self.upper_hsv[1]:.1f}, {self.upper_hsv[2]:.1f}]",
                       (10, y_offset + 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Show geometry calibration points (circular platform)
        # Draw motor attachment points
        colors = [(0, 255, 0), (255, 165, 0), (255, 0, 255)]  # Green, Orange, Magenta
        for i, point in enumerate(self.motor_attachment_points):
            if point:
                color = colors[i % len(colors)]
                cv2.circle(overlay, point, 8, color, -1)
                cv2.putText(overlay, f"M{i+1}", (point[0]+10, point[1]-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw platform center if calculated
        if self.platform_center:
            cv2.circle(overlay, self.platform_center, 8, (0, 255, 255), -1)
            cv2.putText(overlay, "Center", (self.platform_center[0]+10, self.platform_center[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Draw all motor positions if calculated
        if self.motor_positions_pixels[0] and self.motor_positions_pixels[1] and self.motor_positions_pixels[2]:
            colors = [(0, 255, 0), (255, 165, 0), (255, 0, 255)]  # Green, Orange, Magenta
            for i, (pos, angle) in enumerate(zip(self.motor_positions_pixels, self.motor_angles_deg)):
                if pos:
                    cv2.circle(overlay, pos, 6, colors[i], 2)
                    cv2.putText(overlay, f"M{i+1}", (pos[0]+10, pos[1]+10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[i], 1)
                    # Draw line from center to motor
                    if self.platform_center:
                        cv2.line(overlay, self.platform_center, pos, colors[i], 1)
        
        # Draw platform circle if center and radius are calculated
        if self.platform_center and self.platform_radius_pixels:
            radius = int(self.platform_radius_pixels)
            cv2.circle(overlay, self.platform_center, radius, (255, 0, 0), 2)
        
        # Show real-time ball detection if color calibration is complete
        if self.lower_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(self.lower_hsv, dtype=np.uint8)
            upper = np.array(self.upper_hsv, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            
            # Clean up mask
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Find and draw detected ball
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(largest)
                if radius > 5:
                    # Draw detection circle
                    cv2.circle(overlay, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(overlay, (int(x), int(y)), 3, (0, 255, 255), -1)
                    
                    # Show position if geometry calibration is complete
                    if self.pixel_to_meter_ratio:
                        pos = self.detect_ball_position(frame)
                        if pos is not None:
                            cv2.putText(overlay, f"Pos: X={pos[0]:.4f}m, Y={pos[1]:.4f}m",
                                       (int(x)+20, int(y)+20),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        
        return overlay
    
    def run(self):
        """Main calibration loop with interactive GUI."""
        # Initialize camera capture
        self.cap = cv2.VideoCapture(self.CAM_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        # Setup OpenCV window and mouse callback
        cv2.namedWindow("Stewart Platform Calibration")
        cv2.setMouseCallback("Stewart Platform Calibration", self.mouse_callback)
        
        # Attempt servo connections
        self.connect_servos()
        
        # Display instructions
        print("[INFO] Stewart Platform 2D Calibration (Circular Platform)")
        print("Phase 1: Click on ball to sample colors, press 'c' when done")
        print("Phase 2: Click on 3 motor attachment points in order (Motor 1, Motor 2, Motor 3)")
        print("         This will determine: center, radius, axis orientation, and motor positions")
        print("Press 's' to save, 'q' to quit")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Draw overlay and display frame
            display = self.draw_overlay(frame)
            cv2.imshow("Stewart Platform Calibration", display)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                # Quit calibration
                break
            elif key == ord('c') and self.phase == "color":
                # Complete color calibration phase
                if self.hsv_samples:
                    self.phase = "geometry"
                    print("[INFO] Color calibration complete. Click on platform corners.")
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()
        if self.servo_serial:
            self.servo_serial.close()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = StewartPlatformCalibrator()
    calibrator.run()

