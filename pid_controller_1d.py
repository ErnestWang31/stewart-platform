# Simple 1D PID Controller
# For individual motor control

import numpy as np
import time

class PIDController1D:
    """Simple 1D PID controller for individual motor control."""
    
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, output_limit=15.0, integral_deadzone=0.01):
        """Initialize PID controller.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            output_limit: Maximum output value (degrees)
            integral_deadzone: Minimum error magnitude to accumulate integral (meters)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = output_limit
        self.integral_deadzone = integral_deadzone  # Dead zone for integral accumulation
        
        # Controller state
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def update(self, position, dt=None):
        """Update PID controller and return control output.
        
        Args:
            position: Current position (meters)
            dt: Time step (seconds). If None, uses elapsed time since last update
            
        Returns:
            output: Control output (degrees)
        """
        # If all gains are zero, return 0 immediately without updating state
        if self.Kp == 0.0 and self.Ki == 0.0 and self.Kd == 0.0:
            return 0.0
        
        current_time = time.time()
        
        # Calculate time step
        if dt is None:
            if self.prev_time is None:
                dt = 0.033  # Default 30 Hz
            else:
                dt = current_time - self.prev_time
                dt = max(0.001, min(dt, 0.1))  # Clamp between 1ms and 100ms
        else:
            dt = dt
        
        # Calculate error
        error = self.setpoint - position
        
        # Scale error (same as 1D beam balancer)
        error_scale = 100.0
        error_scaled = error * error_scale
        
        # Proportional term
        P = self.Kp * error_scaled
        
        # Derivative term - calculate first (needed for conditional integration)
        if self.Kd > 0 and self.prev_time is not None:
            derivative = (error_scaled - self.prev_error) / dt
            D = self.Kd * derivative
        else:
            derivative = 0.0
            D = 0.0
        
        # Integral term - only accumulate if Ki > 0
        if self.Ki > 0:
            # CRITICAL FIX: Conditional integration to prevent windup
            # Only accumulate integral if:
            # 1. Error is above deadzone threshold (prevents tiny errors from accumulating)
            # 2. Output is not saturated (prevents windup when at limits)
            
            # Calculate what output would be without integral (P + D only)
            output_without_I = P + D
            is_saturated = abs(output_without_I) >= self.output_limit
            
            # Only accumulate if error is significant AND output is not saturated
            if abs(error) >= self.integral_deadzone and not is_saturated:
                self.integral += error_scaled * dt
            # If error is very small, decay the integral slightly to prevent drift
            elif abs(error) < self.integral_deadzone * 0.5:
                # Decay integral by 5% per cycle when error is very small
                self.integral *= 0.95
            
            # Anti-windup: limit integral to prevent excessive buildup
            max_integral = self.output_limit / self.Ki if self.Ki > 0 else 0
            self.integral = np.clip(self.integral, -max_integral, max_integral)
            I = self.Ki * self.integral
        else:
            # Reset integral when Ki is set to 0
            self.integral = 0.0
            I = 0.0
        
        # PID output
        output = P + I + D
        output = np.clip(output, -self.output_limit, self.output_limit)
        
        # Update state only if gains are non-zero
        self.prev_error = error_scaled
        self.prev_time = current_time
        
        return output
    
    def set_setpoint(self, setpoint):
        """Set target position.
        
        Args:
            setpoint: Target position (meters)
        """
        self.setpoint = setpoint
    
    def set_gains(self, Kp, Ki, Kd):
        """Update PID gains.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def reset_integral(self):
        """Reset integral term."""
        self.integral = 0.0

