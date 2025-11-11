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
        self.PLATFORM_RADIUS_M = 0.1  # Known platform radius in meters (circular platform)
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
        self.platform_center = None  # Platform center pixel coordinates (calculated from 3 points)
        self.motor_points = []  # List of 3 motor attachment points [(x1,y1), (x2,y2), (x3,y3)]
        self.motor_angles_deg = []  # Angular positions of motors in degrees (0-360, from +X axis, CCW)
        self.motor_indices = [None, None, None]  # Which clicked point corresponds to Motor 1, 2, 3
        self.platform_radius_pixels = None  # Platform radius in pixels (calculated from 3 points)
        self.pixel_to_meter_ratio = None  # Conversion ratio from pixels to meters (uniform for circle)
        
        # Platform hardware configuration
        # Single Arduino with PWM servo driver controls all 3 servos via one serial port
        self.servo_serial = None  # Single serial connection to Arduino
        self.servo_port = "COM3"  # Default Arduino serial port
        self.neutral_angles = [15, 15, 15]  # Servo neutral position angles
        
        # Position limit results
        self.position_min_x = None  # Minimum ball position in X (meters)
        self.position_max_x = None  # Maximum ball position in X (meters)
        self.position_min_y = None  # Minimum ball position in Y (meters)
        self.position_max_y = None  # Maximum ball position in Y (meters)
        
        # Motor direction testing
        # Load existing motor direction settings from config if available
        self.motor_direction_invert = [False, False, False]
        try:
            import os
            if os.path.exists("config_stewart.json"):
                with open("config_stewart.json", "r") as f:
                    config = json.load(f)
                    if "servo" in config and "motor_direction_invert" in config["servo"]:
                        self.motor_direction_invert = [bool(x) for x in config["servo"]["motor_direction_invert"]]
                        print(f"[CONFIG] Loaded motor directions: {self.motor_direction_invert}")
        except Exception as e:
            print(f"[CONFIG] Could not load motor directions: {e}")
        self.testing_motor = None  # Current motor being tested (0, 1, 2, or None)
    
    def connect_servos(self):
        """Establish serial connection to Arduino with PWM servo driver.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        # Try to load port from existing config
        try:
            import os
            if os.path.exists("config_stewart.json"):
                with open("config_stewart.json", "r") as f:
                    config = json.load(f)
                    if "servo" in config:
                        # Check for single port or ports array
                        if "port" in config["servo"]:
                            self.servo_port = config["servo"]["port"]
                        elif "ports" in config["servo"] and len(config["servo"]["ports"]) > 0:
                            # Use first port if array exists (for backward compatibility)
                            self.servo_port = config["servo"]["ports"][0]
        except Exception as e:
            print(f"[CONFIG] Could not load servo port: {e}")
        
        try:
            self.servo_serial = serial.Serial(self.servo_port, 9600, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            # Flush any initial data
            self.servo_serial.reset_input_buffer()
            print(f"[ARDUINO] Connected to {self.servo_port}")
            return True
        except Exception as e:
            print(f"[ARDUINO] Failed to connect to {self.servo_port}: {e}")
            print("[ARDUINO] Limits will be estimated without servos")
            self.servo_serial = None
            return False
    
    def send_servo_angles(self, angles):
        """Send angle commands to all 3 servos via Arduino PWM driver.
        
        Args:
            angles: List of 3 angles in degrees (0-30)
        """
        if self.servo_serial:
            try:
                # Clip all angles to safe range and convert to bytes
                clipped_angles = [int(np.clip(angle, 0, 30)) for angle in angles]
                # Send all 3 angles as 3 bytes in one write (Arduino expects this format)
                self.servo_serial.write(bytes(clipped_angles))
                self.servo_serial.flush()  # Ensure data is sent immediately
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
                # Collect 3 motor attachment points
                if len(self.motor_points) < 3:
                    self.motor_points.append((x, y))
                    motor_num = len(self.motor_points)
                    print(f"[GEO] Motor attachment point {motor_num} selected at ({x}, {y})")
                    if len(self.motor_points) < 3:
                        print(f"[GEO] Click on motor attachment point {motor_num + 1} (on platform edge)")
                    else:
                        print("[GEO] All 3 points collected. Calculating geometry...")
                        self.calculate_geometry()
    
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
    
    def calculate_circle_from_three_points(self, p1, p2, p3):
        """Calculate circle center and radius from three points.
        
        Args:
            p1, p2, p3: Three points as (x, y) tuples
            
        Returns:
            (center_x, center_y, radius) or None if points are collinear
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        # Calculate circle center using perpendicular bisectors
        # Midpoints
        mid1_x, mid1_y = (x1 + x2) / 2, (y1 + y2) / 2
        mid2_x, mid2_y = (x2 + x3) / 2, (y2 + y3) / 2
        
        # Direction vectors
        d1_x, d1_y = x2 - x1, y2 - y1
        d2_x, d2_y = x3 - x2, y3 - y2
        
        # Perpendicular vectors (swap and negate one component)
        perp1_x, perp1_y = -d1_y, d1_x
        perp2_x, perp2_y = -d2_y, d2_x
        
        # Solve for intersection of perpendicular bisectors
        # Line 1: mid1 + t * perp1
        # Line 2: mid2 + s * perp2
        # mid1 + t * perp1 = mid2 + s * perp2
        
        # Matrix form: [perp1_x  -perp2_x] [t]   [mid2_x - mid1_x]
        #              [perp1_y  -perp2_y] [s] = [mid2_y - mid1_y]
        
        det = perp1_x * (-perp2_y) - perp1_y * (-perp2_x)
        if abs(det) < 1e-6:
            # Points are collinear
            return None
        
        dx = mid2_x - mid1_x
        dy = mid2_y - mid1_y
        
        t = (dx * (-perp2_y) - dy * (-perp2_x)) / det
        
        center_x = mid1_x + t * perp1_x
        center_y = mid1_y + t * perp1_y
        
        # Calculate radius as average distance from center to all three points
        r1 = np.sqrt((x1 - center_x)**2 + (y1 - center_y)**2)
        r2 = np.sqrt((x2 - center_x)**2 + (y2 - center_y)**2)
        r3 = np.sqrt((x3 - center_x)**2 + (y3 - center_y)**2)
        radius = (r1 + r2 + r3) / 3
        
        return (center_x, center_y, radius)
    
    def calculate_geometry(self):
        """Calculate platform geometry from 3 motor attachment points."""
        if len(self.motor_points) != 3:
            return
        
        # Calculate circle from 3 points
        circle_result = self.calculate_circle_from_three_points(
            self.motor_points[0],
            self.motor_points[1],
            self.motor_points[2]
        )
        
        if circle_result is None:
            print("[GEO] ERROR: Points are collinear. Please select 3 non-collinear points.")
            self.motor_points = []  # Reset to try again
            return
        
        center_x, center_y, radius = circle_result
        self.platform_center = (center_x, center_y)
        self.platform_radius_pixels = radius
        
        # Calculate pixel-to-meter conversion ratio
        self.pixel_to_meter_ratio = self.PLATFORM_RADIUS_M / self.platform_radius_pixels
        self.pixel_to_meter_ratio_x = self.pixel_to_meter_ratio
        self.pixel_to_meter_ratio_y = self.pixel_to_meter_ratio
        
        # Calculate angular positions of each motor point (0-360 degrees, from +X axis, CCW)
        self.motor_angles_deg = []
        for px, py in self.motor_points:
            dx = px - center_x
            dy = py - center_y
            # atan2 gives angle from +X axis, CCW is positive
            angle_rad = np.arctan2(dy, dx)
            angle_deg = np.degrees(angle_rad)
            # Normalize to 0-360
            if angle_deg < 0:
                angle_deg += 360
            self.motor_angles_deg.append(angle_deg)
        
        # Identify which point corresponds to Motor 1, 2, 3
        # Expected positions: Motor 1 at 90°, Motor 2 at 210°, Motor 3 at 330°
        expected_angles = [90, 210, 330]
        self.motor_indices = [None, None, None]
        
        # Find closest match for each motor
        for motor_idx, expected_angle in enumerate(expected_angles):
            best_match = None
            best_diff = float('inf')
            for point_idx, actual_angle in enumerate(self.motor_angles_deg):
                # Calculate angular difference (considering wrap-around)
                diff = abs(actual_angle - expected_angle)
                if diff > 180:
                    diff = 360 - diff
                if diff < best_diff and point_idx not in self.motor_indices:
                    best_diff = diff
                    best_match = point_idx
            if best_match is not None:
                self.motor_indices[motor_idx] = best_match
        
        print(f"[GEO] Platform center: ({center_x:.1f}, {center_y:.1f})")
        print(f"[GEO] Platform radius: {radius:.2f} pixels = {self.PLATFORM_RADIUS_M:.4f} meters")
        print(f"[GEO] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
        print(f"[GEO] Motor angles: {[f'{a:.1f}°' for a in self.motor_angles_deg]}")
        print(f"[GEO] Motor assignments: Point {self.motor_indices[0]+1} -> Motor 1 (90°), "
              f"Point {self.motor_indices[1]+1} -> Motor 2 (210°), "
              f"Point {self.motor_indices[2]+1} -> Motor 3 (330°)")
        
        # Advance to complete phase
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
    
    def find_limits_automatically(self):
        """Use servo motors to automatically find ball position limits."""
        if not self.servo_serial:
            # Estimate limits without servos if connection failed
            # Use platform radius as estimate
            self.position_min_x = -self.PLATFORM_RADIUS_M * 0.8
            self.position_max_x = self.PLATFORM_RADIUS_M * 0.8
            self.position_min_y = -self.PLATFORM_RADIUS_M * 0.8
            self.position_max_y = self.PLATFORM_RADIUS_M * 0.8
            print("[LIMITS] Estimated without servos")
            return
        
        print("[LIMITS] Finding limits with servos...")
        positions_x = []
        positions_y = []
        
        # Test platform at different tilt angles to find position range
        # Test angles: neutral, roll left, roll right, pitch forward, pitch backward
        test_configs = [
            ([15, 15, 15], "neutral"),
            ([20, 12, 12], "roll_left"),
            ([10, 18, 18], "roll_right"),
            ([15, 20, 10], "pitch_forward"),
            ([15, 10, 20], "pitch_backward")
        ]
        
        for angles, name in test_configs:
            # Move platform to test configuration
            self.send_servo_angles(angles)
            time.sleep(2)  # Wait for ball to settle
            
            # Collect multiple position measurements
            config_positions = []
            start_time = time.time()
            while time.time() - start_time < 1.0:
                ret, frame = self.cap.read()
                if ret:
                    pos = self.detect_ball_position(frame)
                    if pos is not None:
                        config_positions.append(pos)
                time.sleep(0.05)
            
            # Calculate average position for this configuration
            if config_positions:
                avg_pos_x = np.mean([p[0] for p in config_positions])
                avg_pos_y = np.mean([p[1] for p in config_positions])
                positions_x.append(avg_pos_x)
                positions_y.append(avg_pos_y)
                print(f"[LIMITS] {name}: X={avg_pos_x:.4f}m, Y={avg_pos_y:.4f}m")
        
        # Return platform to neutral position
        if self.testing_motor is None:  # Only return to neutral if not testing
            self.send_servo_angles(self.neutral_angles)
        
        # Determine position limits from collected data
        if len(positions_x) >= 2:
            self.position_min_x = min(positions_x)
            self.position_max_x = max(positions_x)
            self.position_min_y = min(positions_y)
            self.position_max_y = max(positions_y)
            print(f"[LIMITS] X Range: {self.position_min_x:.4f}m to {self.position_max_x:.4f}m")
            print(f"[LIMITS] Y Range: {self.position_min_y:.4f}m to {self.position_max_y:.4f}m")
        else:
            print("[LIMITS] Failed to find limits")
    
    def save_config(self):
        """Save all calibration results to config_stewart.json file.
        
        Preserves existing config fields (like motor_direction_invert) and only updates
        calibration-related fields.
        """
        # Load existing config if it exists, otherwise start with empty dict
        try:
            with open("config_stewart.json", "r") as f:
                config = json.load(f)
        except FileNotFoundError:
            config = {}
        
        # Update timestamp
        config["timestamp"] = datetime.now().isoformat()
        
        # Update platform info
        config["platform_type"] = "circular"
        config["platform_radius_m"] = float(self.PLATFORM_RADIUS_M)
        config["platform_center_pixels"] = list(self.platform_center) if self.platform_center else None
        config["platform_radius_pixels"] = float(self.platform_radius_pixels) if self.platform_radius_pixels else None
        
        # Save motor attachment points and their assignments
        if self.motor_points:
            config["motor_points_pixels"] = [list(p) for p in self.motor_points]
            config["motor_angles_deg"] = [float(a) for a in self.motor_angles_deg] if self.motor_angles_deg else None
            config["motor_indices"] = [int(i) for i in self.motor_indices] if self.motor_indices else None
            # Motor positions in degrees (expected: 90°, 210°, 330°)
            config["motor_positions_deg"] = [90, 210, 330]
        
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
        config["calibration"]["position_min_x_m"] = float(self.position_min_x) if self.position_min_x else None
        config["calibration"]["position_max_x_m"] = float(self.position_max_x) if self.position_max_x else None
        config["calibration"]["position_min_y_m"] = float(self.position_min_y) if self.position_min_y else None
        config["calibration"]["position_max_y_m"] = float(self.position_max_y) if self.position_max_y else None
        
        # Update servo settings (preserve existing fields like motor_direction_invert)
        if "servo" not in config:
            config["servo"] = {}
        config["servo"]["port"] = str(self.servo_port)  # Single port for Arduino
        config["servo"]["neutral_angles"] = [int(a) for a in self.neutral_angles]
        # Save motor direction inversion (from testing or existing config)
        if hasattr(self, 'motor_direction_invert'):
            config["servo"]["motor_direction_invert"] = [bool(x) for x in self.motor_direction_invert]
        elif "motor_direction_invert" not in config["servo"]:
            # Default to all normal if not set
            config["servo"]["motor_direction_invert"] = [False, False, False]
        # Note: motor_direction_invert and other servo fields are preserved if they exist
        
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
        if "platform" not in config:
            config["platform"] = {}
        if "max_roll_angle" not in config["platform"]:
            config["platform"]["max_roll_angle"] = 15.0
        if "max_pitch_angle" not in config["platform"]:
            config["platform"]["max_pitch_angle"] = 15.0
        # Note: Other platform fields (like use_inverse_kinematics, motor_scale_factor, etc.) are preserved
        
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
            "geometry": "Click on 3 motor attachment points on platform edge (one for each motor)",
            "complete": "Calibration complete! Press 's' to save, 'l' to find limits"
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
        
        # Show geometry calibration points (3 motor attachment points)
        motor_colors = [(0, 255, 0), (255, 0, 255), (0, 255, 255)]  # Green, Magenta, Cyan
        motor_labels = ["M1", "M2", "M3"]
        
        for i, point in enumerate(self.motor_points):
            color = motor_colors[i]
            cv2.circle(overlay, point, 10, color, -1)
            cv2.circle(overlay, point, 12, color, 2)
            # Show which motor this point is assigned to
            if self.motor_indices and i in self.motor_indices:
                motor_num = self.motor_indices.index(i) + 1
                label = f"P{i+1}->M{motor_num}"
            else:
                label = f"P{i+1}"
            cv2.putText(overlay, label, (point[0]+15, point[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Draw platform circle if 3 points are selected
        if self.platform_center and self.platform_radius_pixels:
            radius = int(self.platform_radius_pixels)
            cv2.circle(overlay, (int(self.platform_center[0]), int(self.platform_center[1])), 
                      radius, (255, 0, 0), 2)
            # Draw center point
            cv2.circle(overlay, (int(self.platform_center[0]), int(self.platform_center[1])), 
                      8, (0, 255, 0), -1)
            cv2.putText(overlay, "Center", 
                       (int(self.platform_center[0])+10, int(self.platform_center[1])-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Draw lines from center to each motor point with labels
            for i, point in enumerate(self.motor_points):
                color = motor_colors[i]
                cv2.line(overlay, 
                         (int(self.platform_center[0]), int(self.platform_center[1])),
                         point, color, 1)
                if self.motor_angles_deg and i < len(self.motor_angles_deg):
                    angle_text = f"{self.motor_angles_deg[i]:.0f}°"
                    mid_x = int((self.platform_center[0] + point[0]) / 2)
                    mid_y = int((self.platform_center[1] + point[1]) / 2)
                    cv2.putText(overlay, angle_text, (mid_x, mid_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
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
        
        # Show motor testing status
        if self.testing_motor is not None:
            motor_num = self.testing_motor + 1
            direction = "REVERSED" if self.motor_direction_invert[self.testing_motor] else "NORMAL"
            cv2.putText(overlay, f"TESTING Motor {motor_num} - Direction: {direction}",
                       (10, overlay.shape[0] - 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(overlay, "Press 't' for next motor, 'r' to reverse direction",
                       (10, overlay.shape[0] - 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Show final results if limit calibration is complete
        if (self.position_min_x is not None and self.position_max_x is not None and
            self.position_min_y is not None and self.position_max_y is not None):
            # For circular platform, show radial limits
            max_radius = np.sqrt(max(self.position_max_x**2, self.position_min_x**2, 
                                    self.position_max_y**2, self.position_min_y**2))
            y_offset = 20 if self.testing_motor is None else 40
            cv2.putText(overlay, f"X Range: {self.position_min_x:.4f}m to {self.position_max_x:.4f}m",
                       (10, overlay.shape[0] - y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(overlay, f"Y Range: {self.position_min_y:.4f}m to {self.position_max_y:.4f}m",
                       (10, overlay.shape[0] - y_offset + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
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
        print("Phase 2: Click on 3 motor attachment points on the platform edge")
        print("         (One click for each motor - they should be evenly spaced)")
        print("         The system will identify Motor 1 (90°), Motor 2 (210°), Motor 3 (330°)")
        print("Phase 3: Press 'l' to find limits automatically")
        print("         Press 't' to test motor directions (cycle through motors)")
        print("         Press 'r' to reverse current motor direction during testing")
        print("Press 's' to save, 'q' to quit")
        
        # Main calibration loop
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.current_frame = frame
            
            # Test motor if in testing mode
            if self.testing_motor is not None and self.servo_serial:
                # Lift the motor being tested
                test_angles = list(self.neutral_angles)  # Start with neutral
                motor_idx = self.testing_motor
                # Apply direction inversion if set (reverse means decrease angle to lift)
                direction = -1 if self.motor_direction_invert[motor_idx] else 1
                test_angles[motor_idx] = int(self.neutral_angles[motor_idx] + 10 * direction)
                test_angles[motor_idx] = np.clip(test_angles[motor_idx], 0, 30)
                self.send_servo_angles(test_angles)
            elif self.testing_motor is None and self.servo_serial:
                # Return to neutral when not testing
                self.send_servo_angles(self.neutral_angles)
            
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
            elif key == ord('l') and self.phase == "complete":
                # Start automatic limit finding
                self.find_limits_automatically()
            elif key == ord('t') and self.phase == "complete":
                # Test motor directions - cycle through motors 1, 2, 3
                if self.testing_motor is None:
                    self.testing_motor = 0
                    print(f"[TEST] Testing Motor {self.testing_motor + 1} - watch platform direction")
                    print("[TEST] Press 't' to cycle to next motor, 'r' to reverse current motor direction")
                else:
                    self.testing_motor = (self.testing_motor + 1) % 3
                    print(f"[TEST] Testing Motor {self.testing_motor + 1} - watch platform direction")
            elif key == ord('r') and self.phase == "complete" and self.testing_motor is not None:
                # Reverse direction of current motor being tested
                self.motor_direction_invert[self.testing_motor] = not self.motor_direction_invert[self.testing_motor]
                direction = "REVERSED" if self.motor_direction_invert[self.testing_motor] else "NORMAL"
                print(f"[TEST] Motor {self.testing_motor + 1} direction set to {direction}")
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
                self.save_config()
                break
        
        # Clean up resources
        # Return to neutral before closing
        if self.testing_motor is None:
            self.send_servo_angles(self.neutral_angles)
        else:
            self.send_servo_angles(self.neutral_angles)  # Always return to neutral on exit
        
        self.cap.release()
        cv2.destroyAllWindows()
        if self.servo_serial:
            self.servo_serial.close()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = StewartPlatformCalibrator()
    calibrator.run()

