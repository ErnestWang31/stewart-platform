# 1D PID Controller Module
# Simplified 1D PID controller for X-axis only

import numpy as np
import time

class PIDController1D:
    """1D PID controller for single-axis control."""
    
    def __init__(self, Kp=10.0, Ki=0.0, Kd=0.0, output_limit=15.0):
        """Initialize 1D PID controller.
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
            output_limit (float): Maximum output value (degrees)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = output_limit
        
        # Controller state
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def update(self, position, dt=None):
        """Update PID controller and return control output.
        
        Args:
            position (float): Current position (meters)
            dt (float, optional): Time step (seconds). If None, uses elapsed time.
            
        Returns:
            float: Control output (degrees)
        """
        current_time = time.time()
        
        # Calculate time step
        if dt is None:
            if self.prev_time is None:
                dt = 0.01  # Default 100 Hz
            else:
                dt = current_time - self.prev_time
                dt = max(0.001, min(dt, 0.1))  # Clamp between 1ms and 100ms
        
        # Calculate error (scaled for easier tuning)
        error = (self.setpoint - position) * 100  # Convert meters to cm for scaling
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        if self.prev_time is not None:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        D = self.Kd * derivative
        
        # PID output
        output = P + I + D
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        # Update state
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def set_setpoint(self, setpoint):
        """Set target position.
        
        Args:
            setpoint (float): Target position (meters)
        """
        self.setpoint = setpoint
    
    def set_gains(self, Kp, Ki, Kd):
        """Update PID gains.
        
        Args:
            Kp (float): Proportional gain
            Ki (float): Integral gain
            Kd (float): Derivative gain
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def reset_integral(self):
        """Reset integral term."""
        self.integral = 0.0
    
    def reset(self):
        """Reset all controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

