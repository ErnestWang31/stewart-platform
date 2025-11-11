# 2D PID Controller for Stewart Platform
# Provides separate PID controllers for X and Y axes (roll and pitch)
# Adapted from 1D PID controller for 2D platform control

import numpy as np
import time

class PIDController2D:
    """2D PID controller with separate controllers for X and Y axes."""
    
    def __init__(self, Kp_x=10.0, Ki_x=0.0, Kd_x=0.0,
                 Kp_y=10.0, Ki_y=0.0, Kd_y=0.0,
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
        
        # Calculate time step
        if dt is None:
            if self.prev_time_x is None:
                dt = 0.033  # Default 30 Hz
            else:
                dt = current_time - self.prev_time_x
                dt = max(0.001, min(dt, 0.1))  # Clamp between 1ms and 100ms
        
        # Update X-axis controller (roll)
        error_x = self.setpoint_x - position_x
        error_x = error_x * 100  # Scale error for easier tuning
        
        # Proportional term
        P_x = self.Kp_x * error_x
        
        # Integral term
        self.integral_x += error_x * dt
        I_x = self.Ki_x * self.integral_x
        
        # Derivative term
        if self.prev_time_x is not None:
            derivative_x = (error_x - self.prev_error_x) / dt
        else:
            derivative_x = 0.0
        D_x = self.Kd_x * derivative_x
        
        # PID output
        output_x = P_x + I_x + D_x
        output_x = np.clip(output_x, -self.output_limit_x, self.output_limit_x)
        
        # Update X-axis state
        self.prev_error_x = error_x
        self.prev_time_x = current_time
        
        # Update Y-axis controller (pitch)
        error_y = self.setpoint_y - position_y
        error_y = error_y * 100  # Scale error for easier tuning
        
        # Proportional term
        P_y = self.Kp_y * error_y
        
        # Integral term
        self.integral_y += error_y * dt
        I_y = self.Ki_y * self.integral_y
        
        # Derivative term
        if self.prev_time_y is not None:
            derivative_y = (error_y - self.prev_error_y) / dt
        else:
            derivative_y = 0.0
        D_y = self.Kd_y * derivative_y
        
        # PID output
        output_y = P_y + I_y + D_y
        output_y = np.clip(output_y, -self.output_limit_y, self.output_limit_y)
        
        # Update Y-axis state
        self.prev_error_y = error_y
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
    
    def get_state(self):
        """Get current controller state.
        
        Returns:
            dict: Current state including errors, integrals, and outputs
        """
        error_x = self.setpoint_x - (self.prev_error_x / 100 if self.prev_error_x != 0 else 0)
        error_y = self.setpoint_y - (self.prev_error_y / 100 if self.prev_error_y != 0 else 0)
        
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

