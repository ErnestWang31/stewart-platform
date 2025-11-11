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
        self.phase = "color"  # Current phase: "color", "geometry", "motors", "complete"
        
        # Color calibration data
        self.hsv_samples = []  # Collected HSV color samples
        self.lower_hsv = None  # Lower HSV bound for ball detection
        self.upper_hsv = None  # Upper HSV bound for ball detection
        
        # Geometry calibration data (circular platform)
        self.platform_center = None  # Platform center pixel coordinates
        self.platform_edge_point = None  # Platform edge point for radius calculation
        self.platform_radius_pixels = None  # Platform radius in pixels
        self.pixel_to_meter_ratio = None  # Conversion ratio from pixels to meters (uniform for circle)
        
        # Motor position calibration data
        self.motor_positions = [None, None, None]  # Motor positions in pixels [(x1,y1), (x2,y2), (x3,y3)]
        self.motor_angles_deg = [None, None, None]  # Motor angles in degrees from platform center
        self.motor_click_count = 0  # Number of motor positions clicked
        
        # Platform hardware configuration
        self.servos = [None, None, None]  # Serial connections to servos
        self.servo_ports = ["COM3", "COM4", "COM5"]  # Servo communication ports
        self.neutral_angles = [15, 15, 15]  # Servo neutral position angles
        
        # Position limit results
        self.position_min_x = None  # Minimum ball position in X (meters)
        self.position_max_x = None  # Maximum ball position in X (meters)
        self.position_min_y = None  # Minimum ball position in Y (meters)
        self.position_max_y = None  # Maximum ball position in Y (meters)
    
    def connect_servos(self):
        """Establish serial connections to servo motors.
        
        Returns:
            bool: True if at least one connection successful, False otherwise
        """
        connected = False
        for i, port in enumerate(self.servo_ports):
            try:
                self.servos[i] = serial.Serial(port, 9600)
                time.sleep(2)  # Allow time for connection to stabilize
                print(f"[SERVO_{i}] Connected to {port}")
                connected = True
            except:
                print(f"[SERVO_{i}] Failed to connect to {port} - limits will be estimated")
                self.servos[i] = None
        return connected
    
    def send_servo_angles(self, angles):
        """Send angle commands to servo motors with safety clipping.
        
        Args:
            angles: List of 3 angles in degrees
        """
        for i, (servo, angle) in enumerate(zip(self.servos, angles)):
            if servo:
                # Clip angle to safe range and send as byte
                angle = int(np.clip(angle, 0, 30))
                servo.write(bytes([angle]))
    
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
                if self.platform_center is None:
                    # First click: platform center
                    self.platform_center = (x, y)
                    print(f"[GEO] Platform center selected at ({x}, {y})")
                    print("[GEO] Now click on the platform edge to set radius")
                elif self.platform_edge_point is None:
                    # Second click: platform edge point
                    self.platform_edge_point = (x, y)
                    print(f"[GEO] Platform edge selected at ({x}, {y})")
                    self.calculate_geometry()
            elif self.phase == "motors":
                # Motor calibration phase - click on each motor position
                if self.motor_click_count < 3:
                    self.motor_positions[self.motor_click_count] = (x, y)
                    self.motor_click_count += 1
                    print(f"[MOTOR] Motor {self.motor_click_count} position selected at ({x}, {y})")
                    if self.motor_click_count < 3:
                        print(f"[MOTOR] Click on motor {self.motor_click_count + 1} position")
                    else:
                        self.calculate_motor_angles()
                        print("[MOTOR] All motor positions calibrated!")
                        self.phase = "complete"
    
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
    
    def calculate_geometry(self):
        """Calculate pixel-to-meter conversion ratio from circular platform geometry."""
        if self.platform_center is None or self.platform_edge_point is None:
            return
        
        # Calculate radius in pixels
        dx = self.platform_edge_point[0] - self.platform_center[0]
        dy = self.platform_edge_point[1] - self.platform_center[1]
        self.platform_radius_pixels = np.sqrt(dx*dx + dy*dy)
        
        # Calculate pixel-to-meter conversion ratio
        # For a circular platform, the ratio is uniform in all directions
        self.pixel_to_meter_ratio = self.PLATFORM_RADIUS_M / self.platform_radius_pixels
        
        # Store for compatibility with ball detection (X and Y use same ratio for circle)
        self.pixel_to_meter_ratio_x = self.pixel_to_meter_ratio
        self.pixel_to_meter_ratio_y = self.pixel_to_meter_ratio
        
        print(f"[GEO] Platform center: {self.platform_center}")
        print(f"[GEO] Platform radius: {self.platform_radius_pixels:.2f} pixels = {self.PLATFORM_RADIUS_M:.4f} meters")
        print(f"[GEO] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
        
        # Advance to motor calibration phase
        self.phase = "motors"
        print("[MOTOR] Now click on the 3 motor positions (in order: motor 1, motor 2, motor 3)")
        print("[MOTOR] Click on motor 1 position")
    
    def calculate_motor_angles(self):
        """Calculate motor angles from platform center based on clicked positions."""
        if self.platform_center is None:
            print("[MOTOR] Error: Platform center not set")
            return
        
        if None in self.motor_positions:
            print("[MOTOR] Error: Not all motor positions set")
            return
        
        center_x, center_y = self.platform_center
        
        # Calculate angle for each motor position
        # Angle is measured from +X axis (right), counter-clockwise
        # In image coordinates: +X is right, +Y is down
        # atan2(y, x) gives angle from +X axis, but we need to account for image Y being inverted
        for i, (mx, my) in enumerate(self.motor_positions):
            # Calculate vector from center to motor
            dx = mx - center_x
            dy = center_y - my  # Invert Y because image Y increases downward
            
            # Calculate angle in degrees (0° = +X axis, 90° = +Y axis, counter-clockwise)
            angle_rad = math.atan2(dy, dx)
            angle_deg = math.degrees(angle_rad)
            
            # Normalize to 0-360 range
            if angle_deg < 0:
                angle_deg += 360
            
            self.motor_angles_deg[i] = angle_deg
            
            print(f"[MOTOR] Motor {i+1} at ({mx}, {my}): angle = {angle_deg:.1f}°")
        
        # Display motor configuration
        print(f"[MOTOR] Motor angles: M1={self.motor_angles_deg[0]:.1f}°, "
              f"M2={self.motor_angles_deg[1]:.1f}°, M3={self.motor_angles_deg[2]:.1f}°")
    
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
        if not any(self.servos):
            # Estimate limits without servos if connection failed
            self.position_min_x = -self.PLATFORM_SIZE_X_M / 2
            self.position_max_x = self.PLATFORM_SIZE_X_M / 2
            self.position_min_y = -self.PLATFORM_SIZE_Y_M / 2
            self.position_max_y = self.PLATFORM_SIZE_Y_M / 2
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
        
        # Update motor positions and angles
        if "servo" not in config:
            config["servo"] = {}
        if self.motor_positions[0] is not None and self.motor_positions[1] is not None and self.motor_positions[2] is not None:
            config["servo"]["motor_positions_pixels"] = [
                list(self.motor_positions[0]),
                list(self.motor_positions[1]),
                list(self.motor_positions[2])
            ]
            config["servo"]["motor_angles_deg"] = [
                float(self.motor_angles_deg[0]),
                float(self.motor_angles_deg[1]),
                float(self.motor_angles_deg[2])
            ]
            print(f"[SAVE] Motor positions saved: {config['servo']['motor_positions_pixels']}")
            print(f"[SAVE] Motor angles saved: {config['servo']['motor_angles_deg']}")
        
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
        config["servo"]["ports"] = [str(p) for p in self.servo_ports]
        config["servo"]["neutral_angles"] = [int(a) for a in self.neutral_angles]
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
            "geometry": "Click on platform center, then click on platform edge (2 points)",
            "motors": f"Click on motor {self.motor_click_count + 1} position (3 motors total). Press 'm' to skip.",
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
        
        # Show geometry calibration points (circular platform)
        if self.platform_center:
            cv2.circle(overlay, self.platform_center, 8, (0, 255, 0), -1)
            cv2.putText(overlay, "Center", (self.platform_center[0]+10, self.platform_center[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        if self.platform_edge_point:
            cv2.circle(overlay, self.platform_edge_point, 8, (0, 255, 0), -1)
            cv2.putText(overlay, "Edge", (self.platform_edge_point[0]+10, self.platform_edge_point[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw platform circle if center and edge are selected
        if self.platform_center and self.platform_edge_point:
            radius = int(np.sqrt((self.platform_edge_point[0] - self.platform_center[0])**2 + 
                                (self.platform_edge_point[1] - self.platform_center[1])**2))
            cv2.circle(overlay, self.platform_center, radius, (255, 0, 0), 2)
        
        # Show motor positions if calibrated
        motor_colors = [(255, 0, 255), (0, 255, 255), (255, 255, 0)]  # Magenta, Cyan, Yellow
        for i, (pos, angle) in enumerate(zip(self.motor_positions, self.motor_angles_deg)):
            if pos is not None:
                mx, my = pos
                cv2.circle(overlay, (mx, my), 10, motor_colors[i], -1)
                cv2.circle(overlay, (mx, my), 12, motor_colors[i], 2)
                label = f"M{i+1}"
                if angle is not None:
                    label += f" ({angle:.1f}°)"
                cv2.putText(overlay, label, (mx+15, my-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, motor_colors[i], 2)
                # Draw line from center to motor
                if self.platform_center:
                    cv2.line(overlay, self.platform_center, (mx, my), motor_colors[i], 1)
        
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
        
        # Show final results if limit calibration is complete
        if (self.position_min_x is not None and self.position_max_x is not None and
            self.position_min_y is not None and self.position_max_y is not None):
            # For circular platform, show radial limits
            max_radius = np.sqrt(max(self.position_max_x**2, self.position_min_x**2, 
                                    self.position_max_y**2, self.position_min_y**2))
            cv2.putText(overlay, f"X Range: {self.position_min_x:.4f}m to {self.position_max_x:.4f}m",
                       (10, overlay.shape[0] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(overlay, f"Y Range: {self.position_min_y:.4f}m to {self.position_max_y:.4f}m",
                       (10, overlay.shape[0] - 20),
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
        print("Phase 2: Click on platform center, then click on platform edge")
        print("Phase 3: Click on motor positions (motor 1, motor 2, motor 3) - press 'm' to skip")
        print("Phase 4: Press 'l' to find limits automatically (optional)")
        print("Press 's' to save, 'q' to quit")
        print("")
        print("[MOTOR CALIBRATION] Click on the 3 motor positions in order:")
        print("  - Motor 1: First click")
        print("  - Motor 2: Second click")
        print("  - Motor 3: Third click")
        print("  The system will automatically calculate motor angles from platform center.")
        
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
                    print("[INFO] Color calibration complete. Click on platform center, then edge.")
            elif key == ord('m') and self.phase == "motors":
                # Skip motor calibration (optional)
                print("[MOTOR] Motor calibration skipped. Using default motor positions.")
                self.phase = "complete"
            elif key == ord('l') and self.phase == "complete":
                # Start automatic limit finding
                self.find_limits_automatically()
            elif key == ord('s') and (self.phase == "complete" or self.phase == "motors"):
                # Save configuration and exit (allow saving even if motors not calibrated)
                if self.phase == "motors":
                    print("[WARNING] Motor positions not calibrated. Saving without motor calibration.")
                self.save_config()
                break
        
        # Clean up resources
        self.cap.release()
        cv2.destroyAllWindows()
        for servo in self.servos:
            if servo:
                servo.close()

if __name__ == "__main__":
    """Run calibration when script is executed directly."""
    calibrator = StewartPlatformCalibrator()
    calibrator.run()

