# 2D PID Controller for Stewart Platform
# Provides separate PID controllers for X and Y axes (roll and pitch)
# Adapted from 1D PID controller for 2D platform control

import numpy as np
import time

class PIDController2D:
    """2D PID controller with separate controllers for X and Y axes."""
    
    def __init__(self, Kp_x=0.0, Ki_x=0.0, Kd_x=0.0,
                 Kp_y=0.0, Ki_y=0.0, Kd_y=0.0,
                 output_limit_x=15.0, output_limit_y=15.0):
        """Initialize 2D PID controller.
        
        Args:
            Kp_x, Ki_x, Kd_x: PID gains for X axis (roll)
            Kp_y, Ki_y, Kd_y: PID gains for Y axis (pitch)
            output_limit_x, output_limit_y: Maximum output values (degrees)
        """
        # X-axis PID gains
        self.Kp_x = Kp_x
        self.Ki_x = Ki_x
        self.Kd_x = Kd_x
        
        # Y-axis PID gains
        self.Kp_y = Kp_y
        self.Ki_y = Ki_y
        self.Kd_y = Kd_y
        
        # Output limits
        self.output_limit_x = output_limit_x
        self.output_limit_y = output_limit_y
        
        # X-axis controller state
        self.setpoint_x = 0.0
        self.integral_x = 0.0
        self.prev_error_x = 0.0
        self.prev_time_x = None
        
        # Y-axis controller state
        self.setpoint_y = 0.0
        self.integral_y = 0.0
        self.prev_error_y = 0.0
        self.prev_time_y = None
        
        # Position filtering to reduce camera noise (simple exponential moving average)
        self.position_x_filtered = None
        self.position_y_filtered = None
        self.filter_alpha = 0.7  # 0 = heavy filtering, 1 = no filtering (lower = smoother but more lag)
    
    def update(self, position_x, position_y, dt=None):
        """Update PID controllers and return control outputs.
        
        Args:
            position_x: Current X position (meters)
            position_y: Current Y position (meters)
            dt: Time step (seconds). If None, uses elapsed time since last update
            
        Returns:
            output_x: Control output for X axis (degrees, roll)
            output_y: Control output for Y axis (degrees, pitch)
        """
        current_time = time.time()
        
        # Calculate time step for X-axis
        if dt is None:
            if self.prev_time_x is None:
                dt_x = 0.033  # Default 30 Hz
            else:
                dt_x = current_time - self.prev_time_x
                dt_x = max(0.001, min(dt_x, 0.1))  # Clamp between 1ms and 100ms
        else:
            dt_x = dt
        
        # Apply position filtering to reduce camera noise (exponential moving average)
        if self.position_x_filtered is None:
            self.position_x_filtered = position_x
            self.position_y_filtered = position_y
        else:
            self.position_x_filtered = self.filter_alpha * position_x + (1 - self.filter_alpha) * self.position_x_filtered
            self.position_y_filtered = self.filter_alpha * position_y + (1 - self.filter_alpha) * self.position_y_filtered
        
        # Use filtered positions for control
        position_x = self.position_x_filtered
        position_y = self.position_y_filtered
        
        # Update X-axis controller (roll)
        # Control logic: if ball is RIGHT (positive X), tilt LEFT (negative roll) to bring ball back
        # error = setpoint - position: if ball is right (pos), error is negative → negative roll ✓ CORRECT
        error_x = self.setpoint_x - position_x
        
        # Scale error to convert from meters to a normalized range (same as 1D beam balancer)
        # Platform radius is ~0.1m, so errors are typically in range [-0.1, 0.1] meters
        # Scale by 100x to make gains more intuitive (same as 1D controller)
        # This allows gains to be in range [1-10] instead of [0.01-0.1]
        error_scale = 100.0  # Same as 1D beam balancer
        error_x_scaled = error_x * error_scale
        
        # Proportional term
        P_x = self.Kp_x * error_x_scaled
        
        # Integral term (uses same scaling as P and D)
        # Only accumulate integral when Ki > 0 to prevent unbounded growth
        if self.Ki_x > 0:
            self.integral_x += error_x_scaled * dt_x
            # Anti-windup: limit integral to prevent excessive buildup
            max_integral = self.output_limit_x / self.Ki_x
            self.integral_x = np.clip(self.integral_x, -max_integral, max_integral)
            I_x = self.Ki_x * self.integral_x
        else:
            I_x = 0.0
        
        # Derivative term
        if self.prev_time_x is not None:
            derivative_x = (error_x_scaled - self.prev_error_x) / dt_x
        else:
            derivative_x = 0.0
        D_x = self.Kd_x * derivative_x
        
        # PID output
        output_x = P_x + I_x + D_x
        output_x = np.clip(output_x, -self.output_limit_x, self.output_limit_x)
        
        # Update X-axis state (store scaled error)
        self.prev_error_x = error_x_scaled
        self.prev_time_x = current_time
        
        # Update Y-axis controller (pitch)
        # Calculate time step for Y-axis (use its own timestamp)
        if dt is None:
            if self.prev_time_y is None:
                dt_y = 0.033  # Default 30 Hz
            else:
                dt_y = current_time - self.prev_time_y
                dt_y = max(0.001, min(dt_y, 0.1))  # Clamp between 1ms and 100ms
        else:
            dt_y = dt
        
        # Control logic: if ball is FORWARD (positive Y), tilt BACKWARD (negative pitch) to bring ball back
        error_y = self.setpoint_y - position_y
        
        # Scale error (same scale factor as X-axis and 1D beam balancer)
        error_scale = 100.0  # Same as 1D beam balancer
        error_y_scaled = error_y * error_scale
        
        # Proportional term
        P_y = self.Kp_y * error_y_scaled
        
        # Integral term (uses same scaling as P and D)
        # Only accumulate integral when Ki > 0 to prevent unbounded growth
        if self.Ki_y > 0:
            self.integral_y += error_y_scaled * dt_y
            # Anti-windup: limit integral to prevent excessive buildup
            max_integral = self.output_limit_y / self.Ki_y
            self.integral_y = np.clip(self.integral_y, -max_integral, max_integral)
            I_y = self.Ki_y * self.integral_y
        else:
            I_y = 0.0
        
        # Derivative term
        if self.prev_time_y is not None:
            derivative_y = (error_y_scaled - self.prev_error_y) / dt_y
        else:
            derivative_y = 0.0
        D_y = self.Kd_y * derivative_y
        
        # PID output
        output_y = P_y + I_y + D_y
        output_y = np.clip(output_y, -self.output_limit_y, self.output_limit_y)
        
        # Update Y-axis state (store scaled error)
        self.prev_error_y = error_y_scaled
        self.prev_time_y = current_time
        
        return output_x, output_y
    
    def set_setpoint(self, setpoint_x, setpoint_y):
        """Set target position for both axes.
        
        Args:
            setpoint_x: Target X position (meters)
            setpoint_y: Target Y position (meters)
        """
        self.setpoint_x = setpoint_x
        self.setpoint_y = setpoint_y
    
    def set_gains_x(self, Kp, Ki, Kd):
        """Update PID gains for X axis.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
        """
        self.Kp_x = Kp
        self.Ki_x = Ki
        self.Kd_x = Kd
    
    def set_gains_y(self, Kp, Ki, Kd):
        """Update PID gains for Y axis.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
        """
        self.Kp_y = Kp
        self.Ki_y = Ki
        self.Kd_y = Kd
    
    def reset_integral(self):
        """Reset integral terms for both axes."""
        self.integral_x = 0.0
        self.integral_y = 0.0
        print("[PID_2D] Integral terms reset")
    
    def reset_integral_x(self):
        """Reset integral term for X axis only."""
        self.integral_x = 0.0
        print("[PID_2D] X-axis integral term reset")
    
    def reset_integral_y(self):
        """Reset integral term for Y axis only."""
        self.integral_y = 0.0
        print("[PID_2D] Y-axis integral term reset")
    
    def set_filter_alpha(self, alpha):
        """Set position filter strength.
        
        Args:
            alpha: Filter alpha value (0 = heavy filtering/smooth but laggy, 1 = no filtering/responsive but noisy)
        """
        self.filter_alpha = np.clip(alpha, 0.0, 1.0)
        print(f"[PID_2D] Position filter alpha set to {self.filter_alpha:.2f}")
    
    def get_state(self):
        """Get current controller state.
        
        Returns:
            dict: Current state including errors, integrals, and outputs
        """
        # prev_error_x and prev_error_y store scaled errors (error * 100.0)
        # To get actual position error, divide by 100.0
        # For now, just return the stored scaled values as-is
        error_x = self.prev_error_x if self.prev_time_x is not None else 0.0
        error_y = self.prev_error_y if self.prev_time_y is not None else 0.0
        
        return {
            'setpoint_x': self.setpoint_x,
            'setpoint_y': self.setpoint_y,
            'error_x': error_x,
            'error_y': error_y,
            'integral_x': self.integral_x,
            'integral_y': self.integral_y,
            'prev_error_x': self.prev_error_x,
            'prev_error_y': self.prev_error_y
        }

