# Ball Tracker Module
# Provides ball position interface for simulation and real hardware

import numpy as np
import time

class SimulatedBallTracker:
    """Simulated ball tracker with simple dynamics model."""
    
    def __init__(self, initial_position=0.10, g=9.81, damping=0.1):
        """Initialize simulated ball tracker.
        
        Args:
            initial_position (float): Initial ball position in meters (default 0.10m = 10cm)
            g (float): Gravitational acceleration (m/s^2)
            damping (float): Damping coefficient for ball motion
        """
        self.position = initial_position
        self.velocity = 0.0
        self.g = g
        self.damping = damping
        self.last_update_time = None
    
    def update(self, tilt_angle_deg, dt=None):
        """Update ball position based on platform tilt.
        
        Args:
            tilt_angle_deg (float): Platform tilt angle in degrees
            dt (float, optional): Time step in seconds. If None, uses elapsed time.
            
        Returns:
            float: Current ball position (meters)
        """
        current_time = time.time()
        
        if dt is None:
            if self.last_update_time is None:
                dt = 0.01  # Default 100 Hz
            else:
                dt = current_time - self.last_update_time
                dt = max(0.001, min(dt, 0.1))  # Clamp between 1ms and 100ms
        
        self.last_update_time = current_time
        
        # Convert tilt angle to radians
        tilt_rad = np.radians(tilt_angle_deg)
        
        # Simple ball dynamics: x_dot = g * sin(θ) - damping * velocity
        # When platform tilts right (positive angle), ball rolls right (positive acceleration)
        # For small angles: sin(θ) ≈ θ, but we'll use full sin for accuracy
        acceleration = self.g * np.sin(tilt_rad) - self.damping * self.velocity
        
        # Update velocity using Euler integration
        self.velocity += acceleration * dt
        
        # Update position
        self.position += self.velocity * dt
        
        return self.position
    
    def get_position(self):
        """Get current ball position.
        
        Returns:
            float: Current ball position (meters)
        """
        return self.position
    
    def reset(self, position=0.10):
        """Reset ball to initial position.
        
        Args:
            position (float): Position to reset to (meters)
        """
        self.position = position
        self.velocity = 0.0
        self.last_update_time = None


class RealBallTracker:
    """Real ball tracker using camera and ball detection."""
    
    def __init__(self, config_file="config_stewart.json", use_x_axis=True):
        """Initialize real ball tracker.
        
        Args:
            config_file (str): Path to configuration file
            use_x_axis (bool): If True, use X-axis position; if False, use Y-axis
        """
        from ball_detection_2d import BallDetector2D
        import cv2
        
        self.detector = BallDetector2D(config_file)
        self.use_x_axis = use_x_axis
        self.cap = None
        self.last_position = 0.0
    
    def initialize_camera(self, camera_index=0):
        """Initialize camera for ball detection.
        
        Args:
            camera_index (int): Camera index
        """
        import cv2
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
    
    def get_position(self):
        """Get current ball position from camera.
        
        Returns:
            float: Current ball position (meters), or last known position if not detected
        """
        if self.cap is None:
            raise RuntimeError("Camera not initialized. Call initialize_camera() first.")
        
        ret, frame = self.cap.read()
        if not ret:
            return self.last_position
        
        found, center, radius, position_x_m, position_y_m = self.detector.detect_ball(frame)
        
        if found:
            if self.use_x_axis:
                self.last_position = position_x_m
            else:
                self.last_position = position_y_m
        
        return self.last_position
    
    def close(self):
        """Close camera connection."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

