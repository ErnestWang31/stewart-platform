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
        self.phase = "color"  # Current phase: "color", "motor_calibration", "complete"
        
        # Color calibration data
        self.hsv_samples = []  # Collected HSV color samples
        self.lower_hsv = None  # Lower HSV bound for ball detection
        self.upper_hsv = None  # Upper HSV bound for ball detection
        
        # Platform geometry (calculated from motor calibration)
        self.platform_center = None  # Platform center pixel coordinates (from motor calibration)
        self.platform_radius_pixels = None  # Platform radius in pixels (from motor calibration)
        self.pixel_to_meter_ratio = None  # Conversion ratio from pixels to meters (uniform for circle)
        
        # Motor calibration data (3-point calibration)
        self.motor_positions = []  # List of 3 motor positions in pixels [(x1,y1), (x2,y2), (x3,y3)]
        self.motor_angles_deg = None  # Angular positions of motors in degrees [angle1, angle2, angle3]
        self.motor_offset_deg = None  # Offset angle for motor 1
        
        # Platform hardware configuration
        self.servos = [None, None, None]  # Serial connections to servos
        self.servo_ports = ["COM3", "COM4", "COM5"]  # Servo communication ports
        self.neutral_angles = [15, 15, 15]  # Servo neutral position angles
        self.servo_baud = 115200
        self.servo_timeout = 1.0
        self.servo_write_timeout = 0.05
        
    
    def connect_servos(self):
        """Establish serial connections to servo motors.
        
        Returns:
            bool: True if at least one connection successful, False otherwise
        """
        connected = False
        for i, port in enumerate(self.servo_ports):
            try:
                self.servos[i] = serial.Serial(
                    port,
                    self.servo_baud,
                    timeout=self.servo_timeout,
                    write_timeout=self.servo_write_timeout
                )
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
            elif self.phase == "motor_calibration":
                # Motor calibration phase - collect 3 motor positions
                if len(self.motor_positions) < 3:
                    self.motor_positions.append((x, y))
                    motor_num = len(self.motor_positions)
                    print(f"[MOTOR] Motor {motor_num} selected at ({x}, {y})")
                    if len(self.motor_positions) < 3:
                        print(f"[MOTOR] Click on motor {motor_num + 1}")
                    else:
                        print("[MOTOR] All 3 motors selected, calculating calibration...")
                        self.calculate_motor_calibration()
    
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
    
    def calculate_motor_calibration(self):
        """Calculate motor positions and angles from 3-point calibration.
        
        Uses 3 motor positions to:
        1. Calculate platform center (circumcenter of triangle formed by 3 motors)
        2. Calculate platform radius (average distance from center to motors)
        3. Calculate angular positions of motors relative to motor 1
        4. Determine motor 1 offset angle
        """
        if len(self.motor_positions) != 3:
            return
        
        # Extract motor positions
        p1 = np.array(self.motor_positions[0])
        p2 = np.array(self.motor_positions[1])
        p3 = np.array(self.motor_positions[2])
        
        # Calculate circumcenter of triangle (platform center)
        # Using formula for circumcenter: intersection of perpendicular bisectors
        # Vector from p1 to p2 and p1 to p3
        v12 = p2 - p1
        v13 = p3 - p1
        
        # Check if points are collinear
        cross_product = v12[0] * v13[1] - v12[1] * v13[0]
        if abs(cross_product) < 1e-6:
            # Points are collinear, use centroid as fallback
            center = (p1 + p2 + p3) / 3.0
            print("[MOTOR] Warning: Motors appear collinear, using centroid")
        else:
            # Calculate perpendicular bisectors
            # Midpoint of edge p1-p2
            mid12 = (p1 + p2) / 2.0
            # Midpoint of edge p1-p3
            mid13 = (p1 + p3) / 2.0
            
            # Direction vectors perpendicular to edges (rotated 90 degrees)
            # Perpendicular to v12: rotate by 90 degrees
            dir12 = np.array([-v12[1], v12[0]])
            # Perpendicular to v13: rotate by 90 degrees
            dir13 = np.array([-v13[1], v13[0]])
            
            # Normalize direction vectors
            norm12 = np.linalg.norm(dir12)
            norm13 = np.linalg.norm(dir13)
            
            if norm12 < 1e-6 or norm13 < 1e-6:
                # Degenerate case, use centroid
                center = (p1 + p2 + p3) / 3.0
                print("[MOTOR] Warning: Degenerate triangle, using centroid")
            else:
                dir12 = dir12 / norm12
                dir13 = dir13 / norm13
                
                # Solve for intersection of perpendicular bisectors
                # Line 1: mid12 + t * dir12
                # Line 2: mid13 + s * dir13
                # Find t such that mid12 + t*dir12 = mid13 + s*dir13
                # Rearranged: t*dir12 - s*dir13 = mid13 - mid12
                A = np.array([[dir12[0], -dir13[0]], [dir12[1], -dir13[1]]])
                b_vec = mid13 - mid12
                
                try:
                    # Solve for t
                    t = np.linalg.solve(A, b_vec)[0]
                    center = mid12 + t * dir12
                except np.linalg.LinAlgError:
                    # Fallback to centroid if solve fails
                    center = (p1 + p2 + p3) / 3.0
                    print("[MOTOR] Warning: Solve failed, using centroid")
        
        # Update platform center
        self.platform_center = tuple(center.astype(int))
        
        # Calculate radius as average distance from center to motors
        r1 = np.linalg.norm(p1 - center)
        r2 = np.linalg.norm(p2 - center)
        r3 = np.linalg.norm(p3 - center)
        self.platform_radius_pixels = (r1 + r2 + r3) / 3.0
        
        # Recalculate pixel-to-meter ratio if we have a known physical radius
        if self.PLATFORM_RADIUS_M > 0:
            self.pixel_to_meter_ratio = self.PLATFORM_RADIUS_M / self.platform_radius_pixels
            self.pixel_to_meter_ratio_x = self.pixel_to_meter_ratio
            self.pixel_to_meter_ratio_y = self.pixel_to_meter_ratio
        
        # Calculate angular positions of motors relative to center
        # Angle measured from positive X axis, counter-clockwise
        # Note: Controller expects -90° to be "up" (0° in controller = -90° in calibration)
        def angle_from_center(point):
            """Calculate angle in degrees from center to point."""
            dx = point[0] - center[0]
            dy = point[1] - center[1]
            angle_rad = np.arctan2(dy, dx)
            angle_deg = np.degrees(angle_rad)
            # Normalize to 0-360
            if angle_deg < 0:
                angle_deg += 360
            return angle_deg
        
        angle1 = angle_from_center(p1)
        angle2 = angle_from_center(p2)
        angle3 = angle_from_center(p3)
        
        # Motor 1 offset is its angle (this is the reference)
        self.motor_offset_deg = angle1
        
        # Calculate relative angles (motor 1 is at offset, others relative to it)
        # Normalize so motor 1 is at 0 degrees (after applying offset)
        motor1_angle = 0.0  # Motor 1 is the reference
        motor2_angle = angle2 - angle1
        motor3_angle = angle3 - angle1
        
        # Normalize angles to 0-360
        if motor2_angle < 0:
            motor2_angle += 360
        if motor3_angle < 0:
            motor3_angle += 360
        
        # Store absolute angles (for reference) and relative angles
        self.motor_angles_deg = [angle1, angle2, angle3]
        
        print(f"[MOTOR] Platform center: ({center[0]:.1f}, {center[1]:.1f})")
        print(f"[MOTOR] Platform radius: {self.platform_radius_pixels:.2f} pixels")
        print(f"[MOTOR] Motor 1 angle: {angle1:.2f}° (offset: {self.motor_offset_deg:.2f}°)")
        print(f"[MOTOR] Motor 2 angle: {angle2:.2f}° (relative: {motor2_angle:.2f}°)")
        print(f"[MOTOR] Motor 3 angle: {angle3:.2f}° (relative: {motor3_angle:.2f}°)")
        print(f"[MOTOR] Pixel-to-meter ratio: {self.pixel_to_meter_ratio:.6f} m/pixel")
        
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
        
        # Update motor calibration data (3-point calibration)
        if len(self.motor_positions) == 3:
            config["motor_positions_pixels"] = [[int(p[0]), int(p[1])] for p in self.motor_positions]
            if self.motor_angles_deg:
                config["motor_angles_deg"] = [float(a) for a in self.motor_angles_deg]
            if self.motor_offset_deg is not None:
                config["axis_rotation_deg"] = float(self.motor_offset_deg)
        
        # Update servo settings (preserve existing fields like motor_direction_invert)
        if "servo" not in config:
            config["servo"] = {}
        config["servo"]["ports"] = [str(p) for p in self.servo_ports]
        config["servo"]["neutral_angles"] = [int(a) for a in self.neutral_angles]
        config["servo"]["baud_rate"] = int(self.servo_baud)
        config["servo"]["write_timeout_ms"] = int(self.servo_write_timeout * 1000)
        if "timeout_seconds" not in config["servo"]:
            config["servo"]["timeout_seconds"] = float(self.servo_timeout)
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
            "motor_calibration": f"Click on 3 motors ({len(self.motor_positions)}/3 clicked)",
            "complete": "Calibration complete! Press 's' to save, 'q' to quit"
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
        
        # Show platform center and circle (from motor calibration)
        if self.platform_center and self.platform_radius_pixels:
            radius = int(self.platform_radius_pixels)
            cv2.circle(overlay, self.platform_center, radius, (255, 0, 0), 2)
            cv2.circle(overlay, self.platform_center, 8, (0, 255, 0), -1)
            cv2.putText(overlay, "Center", (self.platform_center[0]+10, self.platform_center[1]-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show motor calibration points
        motor_colors = [(255, 0, 255), (0, 255, 255), (255, 255, 0)]  # Magenta, Cyan, Yellow
        for i, motor_pos in enumerate(self.motor_positions):
            color = motor_colors[i % len(motor_colors)]
            cv2.circle(overlay, motor_pos, 10, color, -1)
            cv2.circle(overlay, motor_pos, 12, color, 2)
            cv2.putText(overlay, f"M{i+1}", (motor_pos[0]+15, motor_pos[1]+5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            # Draw line from center to motor if center is known
            if self.platform_center:
                cv2.line(overlay, self.platform_center, motor_pos, color, 1)
        
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
                    
                    # Show position if calibration is complete
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
        print("Phase 2: Click on 3 motors (any order) for motor calibration")
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
                    self.phase = "motor_calibration"
                    print("[INFO] Color calibration complete. Click on the 3 motors (any order).")
            elif key == ord('s') and self.phase == "complete":
                # Save configuration and exit
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

