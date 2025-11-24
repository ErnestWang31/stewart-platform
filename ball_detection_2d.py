# 2D Ball Detection Module for Stewart Platform
# Detects colored balls in video frames and returns (x, y) position
# Adapted from 1D ball detection for 2D platform control

import cv2
import numpy as np
import json
import os
import math

class BallDetector2D:
    """Computer vision ball detector using HSV color space filtering for 2D position tracking."""
    
    def __init__(self, config_file="config_stewart.json"):
        """Initialize ball detector with HSV bounds from config file.
        
        Args:
            config_file (str): Path to JSON config file with HSV bounds and calibration
        """
        # Default HSV bounds for orange ball detection
        self.lower_hsv = np.array([5, 150, 150], dtype=np.uint8)  # Orange lower bound
        self.upper_hsv = np.array([20, 255, 255], dtype=np.uint8)  # Orange upper bound
        self.scale_factor_x = 1.0  # Conversion factor from normalized x-coords to meters
        self.scale_factor_y = 1.0  # Conversion factor from normalized y-coords to meters
        self.config = None  # Store config for platform center access
        self.axis_rotation_deg = 0.0
        self.axis_rotation_rad = 0.0
        
        # Load configuration from file if it exists
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    self.config = config  # Store config for later use
                
                # Extract HSV color bounds from config
                if 'ball_detection' in config:
                    if config['ball_detection']['lower_hsv']:
                        self.lower_hsv = np.array(config['ball_detection']['lower_hsv'], dtype=np.uint8)
                    if config['ball_detection']['upper_hsv']:
                        self.upper_hsv = np.array(config['ball_detection']['upper_hsv'], dtype=np.uint8)
                
                # Extract scale factors for position conversion from pixels to meters
                if 'calibration' in config:
                    if 'pixel_to_meter_ratio_x' in config['calibration']:
                        frame_width = config.get('camera', {}).get('frame_width', 640)
                        self.scale_factor_x = config['calibration']['pixel_to_meter_ratio_x'] * (frame_width / 2)
                    if 'pixel_to_meter_ratio_y' in config['calibration']:
                        frame_height = config.get('camera', {}).get('frame_height', 480)
                        self.scale_factor_y = config['calibration']['pixel_to_meter_ratio_y'] * (frame_height / 2)
                    # Fallback to single ratio if separate ones not provided
                    if 'pixel_to_meter_ratio' in config['calibration'] and self.scale_factor_x == 1.0:
                        frame_width = config.get('camera', {}).get('frame_width', 640)
                        frame_height = config.get('camera', {}).get('frame_height', 480)
                        ratio = config['calibration']['pixel_to_meter_ratio']
                        self.scale_factor_x = ratio * (frame_width / 2)
                        self.scale_factor_y = ratio * (frame_height / 2)
                
                if config.get("axis_rotation_deg") is not None:
                    self.axis_rotation_deg = float(config["axis_rotation_deg"])
                    self.axis_rotation_rad = math.radians(self.axis_rotation_deg)

                print(f"[BALL_DETECT_2D] Loaded HSV bounds: {self.lower_hsv} to {self.upper_hsv}")
                print(f"[BALL_DETECT_2D] Scale factors: X={self.scale_factor_x:.6f}, Y={self.scale_factor_y:.6f} m/normalized_unit")
                if self.axis_rotation_deg:
                    print(f"[BALL_DETECT_2D] Axis rotation: {self.axis_rotation_deg:.2f}°")
                
            except Exception as e:
                print(f"[BALL_DETECT_2D] Config load error: {e}, using defaults")
        else:
            print("[BALL_DETECT_2D] No config file found, using default HSV bounds")

    def detect_ball(self, frame, rotate_axis=False):
        """Detect ball in frame and return detection results.
        
        Args:
            frame: Input BGR image frame
            
        Returns:
            found (bool): True if ball detected
            center (tuple): (x, y) pixel coordinates of ball center
            radius (float): Ball radius in pixels
            position_x_m (float): Ball x position in meters from center
            position_y_m (float): Ball y position in meters from center
        """
        # Convert frame from BGR to HSV color space for better color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create binary mask using HSV color bounds
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # Clean up mask using morphological operations
        mask = cv2.erode(mask, None, iterations=2)  # Remove noise
        mask = cv2.dilate(mask, None, iterations=2)  # Fill gaps
        
        # Find all contours in the cleaned mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, None, None, 0.0, 0.0
        
        # Select the largest contour (assumed to be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get minimum enclosing circle around the contour
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
        # Filter out detections that are too small or too large
        if radius < 5 or radius > 100:
            return False, None, None, 0.0, 0.0
        
        # Convert pixel position to meters from platform center
        # Try to use calibrated platform center from config, otherwise use frame center
        center_x = frame.shape[1] // 2  # Default: frame center x-coordinate
        center_y = frame.shape[0] // 2  # Default: frame center y-coordinate
        
        # Check if config has platform center (for circular platform calibration)
        if hasattr(self, 'config') and self.config:
            platform_center = self.config.get('platform_center_pixels')
            if platform_center and len(platform_center) == 2:
                center_x, center_y = platform_center[0], platform_center[1]
        
        pixel_offset_x = x - center_x
        pixel_offset_y = y - center_y
        
        # For circular platform, use uniform scale factor
        if self.scale_factor_x == self.scale_factor_y or abs(self.scale_factor_x - self.scale_factor_y) < 1e-6:
            # Uniform scale (circular platform)
            scale = self.scale_factor_x
            position_x_m = pixel_offset_x * scale
            position_y_m = pixel_offset_y * scale
        else:
            # Non-uniform scale (rectangular platform)
            normalized_x = pixel_offset_x / (frame.shape[1] // 2) if frame.shape[1] // 2 > 0 else 0
            normalized_y = pixel_offset_y / (frame.shape[0] // 2) if frame.shape[0] // 2 > 0 else 0
            position_x_m = normalized_x * self.scale_factor_x
            position_y_m = normalized_y * self.scale_factor_y
        
        if rotate_axis and self.axis_rotation_rad:
            cos_r = math.cos(self.axis_rotation_rad)
            sin_r = math.sin(self.axis_rotation_rad)
            rotated_x = position_x_m * cos_r + position_y_m * sin_r
            rotated_y = -position_x_m * sin_r + position_y_m * cos_r
            position_x_m, position_y_m = rotated_x, rotated_y

        return True, (int(x), int(y)), radius, position_x_m, position_y_m

    def draw_detection(self, frame, show_info=True, axis_overlay=None):
        """Detect ball and draw detection overlay on frame.
        
        Args:
            frame: Input BGR image frame
            show_info (bool): Whether to display position information text
            
        Returns:
            frame_with_overlay: Frame with detection drawn
            found: True if ball detected
            position_x_m: Ball x position in meters
            position_y_m: Ball y position in meters
        """
        # Perform ball detection
        found, center, radius, position_x_m, position_y_m = self.detect_ball(frame)
        
        # Create overlay copy for drawing
        overlay = frame.copy()
        
        # Draw center reference cross (use platform center if calibrated)
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Check if config has platform center (for circular platform calibration)
        axis_dir = axis_overlay
        platform_radius_pixels = None
        if self.config and self.config.get('platform_center_pixels'):
            platform_center = self.config.get('platform_center_pixels')
            if platform_center and len(platform_center) == 2:
                center_x, center_y = int(platform_center[0]), int(platform_center[1])
                # Also draw platform circle if radius is available
                if self.config.get('platform_radius_pixels'):
                    platform_radius_pixels = int(self.config.get('platform_radius_pixels'))
                    cv2.circle(overlay, (center_x, center_y), platform_radius_pixels, (200, 200, 200), 1)
        radius_pixels = platform_radius_pixels or int(min(height, width) // 3)
        
        cv2.line(overlay, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 1)
        cv2.line(overlay, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 1)
        cv2.putText(overlay, "Center", (center_x + 5, center_y - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Draw selected axis overlay if requested
        if axis_dir in ("x", "roll", "horizontal", "y", "pitch", "vertical"):
            roll_angle_rad = -self.axis_rotation_rad
            pitch_angle_rad = roll_angle_rad + math.pi / 2.0

            def draw_axis(angle_rad, color, label_text):
                dx = math.cos(angle_rad)
                dy = math.sin(angle_rad)
                x1 = int(center_x - dx * radius_pixels)
                y1 = int(center_y - dy * radius_pixels)
                x2 = int(center_x + dx * radius_pixels)
                y2 = int(center_y + dy * radius_pixels)
                cv2.line(overlay, (x1, y1), (x2, y2), color, 2)
                label_pos = (int(center_x + dx * (radius_pixels * 0.5)),
                             int(center_y + dy * (radius_pixels * 0.5)))
                cv2.putText(overlay, label_text, label_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if axis_dir in ("x", "roll", "horizontal"):
                draw_axis(roll_angle_rad, (0, 0, 255), "Roll Axis")
            else:
                draw_axis(pitch_angle_rad, (255, 0, 0), "Pitch Axis")
        
        if found:
            # Draw circle around detected ball
            cv2.circle(overlay, center, int(radius), (0, 255, 0), 2)  # Green circle
            cv2.circle(overlay, center, 3, (0, 255, 0), -1)  # Green center dot
            
            # Draw line from center to ball
            cv2.line(overlay, (center_x, center_y), center, (0, 255, 255), 1)
            
            if show_info:
                # Display ball position information
                cv2.putText(overlay, f"x: {center[0]}, y: {center[1]}", 
                           (center[0] - 40, center[1] - 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(overlay, f"X: {position_x_m:.4f}m", 
                           (center[0] - 40, center[1] - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(overlay, f"Y: {position_y_m:.4f}m", 
                           (center[0] - 40, center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return overlay, found, position_x_m, position_y_m

# Legacy function for backward compatibility (returns normalized coordinates)
def detect_ball_xy(frame):
    """Legacy function that matches the old ball_detection.py interface.
    
    Args:
        frame: Input BGR image frame
        
    Returns:
        found (bool): True if ball detected
        x_normalized (float): Normalized x position (-1 to +1)
        y_normalized (float): Normalized y position (-1 to +1)
        vis_frame (array): Frame with detection overlay
    """
    # Create detector instance using default config
    detector = BallDetector2D()
    
    # Get detection results with visual overlay
    vis_frame, found, position_x_m, position_y_m = detector.draw_detection(frame)
    
    if found:
        # Convert back to normalized coordinates for legacy compatibility
        x_normalized = position_x_m / detector.scale_factor_x if detector.scale_factor_x != 0 else 0.0
        y_normalized = position_y_m / detector.scale_factor_y if detector.scale_factor_y != 0 else 0.0
        x_normalized = np.clip(x_normalized, -1.0, 1.0)  # Ensure within bounds
        y_normalized = np.clip(y_normalized, -1.0, 1.0)  # Ensure within bounds
    else:
        x_normalized = 0.0
        y_normalized = 0.0
    
    return found, x_normalized, y_normalized, vis_frame

# For testing/calibration when run directly
def main():
    """Test 2D ball detection with current config."""
    detector = BallDetector2D()
    cap = cv2.VideoCapture(0)  # Use default camera
    
    print("2D Ball Detection Test for Stewart Platform")
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Resize frame for consistent processing
        frame = cv2.resize(frame, (640, 480))
        
        # Get detection results with overlay
        vis_frame, found, position_x_m, position_y_m = detector.draw_detection(frame)
        
        # Show detection info in console
        if found:
            print(f"Ball detected at X: {position_x_m:.4f}m, Y: {position_y_m:.4f}m from center")
        
        # Display frame with detection overlay
        cv2.imshow("2D Ball Detection Test", vis_frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

