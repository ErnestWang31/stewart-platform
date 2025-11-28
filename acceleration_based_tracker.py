# Acceleration-Based Trajectory Tracking Module
# Computes commanded acceleration from trajectory and state feedback
# Converts acceleration to actuator commands via pluggable mappings

import numpy as np


class ActuatorMapping:
    """Base class for actuator mappings that convert acceleration to actuator commands."""
    
    def acceleration_to_command(self, acceleration):
        """Convert commanded acceleration to actuator command.
        
        Args:
            acceleration (float): Commanded acceleration (m/s²)
            
        Returns:
            float: Actuator command (units depend on mapping)
        """
        raise NotImplementedError("Subclasses must implement acceleration_to_command")


class TiltAngleMapping(ActuatorMapping):
    """Mapping from acceleration to tilt angle for Stewart platform.
    
    For a ball on a tilted platform: xdd = g * sin(θ)
    Therefore: θ = arcsin(xdd / g)
    For small angles: θ ≈ xdd / g
    """
    
    def __init__(self, g=9.81, use_small_angle_approx=False):
        """Initialize tilt angle mapping.
        
        Args:
            g (float): Gravitational acceleration (m/s²)
            use_small_angle_approx (bool): If True, use θ ≈ xdd/g; if False, use arcsin
        """
        self.g = g
        self.use_small_angle_approx = use_small_angle_approx
    
    def acceleration_to_command(self, acceleration):
        """Convert acceleration to tilt angle in degrees.
        
        Args:
            acceleration (float): Commanded acceleration (m/s²)
            
        Returns:
            float: Tilt angle in degrees
        """
        # Clamp acceleration to physical limits
        max_accel = self.g  # Can't exceed gravity
        acceleration = np.clip(acceleration, -max_accel, max_accel)
        
        if self.use_small_angle_approx:
            # Small angle approximation: θ ≈ xdd / g
            tilt_rad = acceleration / self.g
        else:
            # Full nonlinear: θ = arcsin(xdd / g)
            tilt_rad = np.arcsin(acceleration / self.g)
        
        # Convert to degrees
        return np.degrees(tilt_rad)


class TorqueMapping(ActuatorMapping):
    """Mapping from acceleration to torque command.
    
    For systems where torque directly controls acceleration: τ = I * xdd
    """
    
    def __init__(self, inertia=1.0):
        """Initialize torque mapping.
        
        Args:
            inertia (float): Moment of inertia (kg·m²)
        """
        self.inertia = inertia
    
    def acceleration_to_command(self, acceleration):
        """Convert acceleration to torque.
        
        Args:
            acceleration (float): Commanded acceleration (m/s²)
            
        Returns:
            float: Torque command (N·m)
        """
        return self.inertia * acceleration


class CurrentMapping(ActuatorMapping):
    """Mapping from acceleration to current command.
    
    For systems where current directly controls acceleration: I = k * xdd
    """
    
    def __init__(self, gain=1.0):
        """Initialize current mapping.
        
        Args:
            gain (float): Current-to-acceleration gain (A·s²/m)
        """
        self.gain = gain
    
    def acceleration_to_command(self, acceleration):
        """Convert acceleration to current.
        
        Args:
            acceleration (float): Commanded acceleration (m/s²)
            
        Returns:
            float: Current command (A)
        """
        return self.gain * acceleration


class AccelerationBasedTracker:
    """Acceleration-based trajectory tracking controller.
    
    Computes commanded acceleration using:
        xdd_cmd = xdd_d + Kp * e + Kd * ed
    
    Where:
        e = x_d - x (position error)
        ed = xd_d - xd (velocity error)
    """
    
    def __init__(self, Kp=10.0, Kd=5.0, 
                 acceleration_limit=None,
                 actuator_mapping=None,
                 g=9.81):
        """Initialize acceleration-based tracker.
        
        Args:
            Kp (float): Proportional gain for position error
            Kd (float): Derivative gain for velocity error
            acceleration_limit (float, optional): Maximum acceleration (m/s²). 
                                                 If None, uses g (gravity)
            actuator_mapping (ActuatorMapping, optional): Mapping from acceleration 
                                                         to actuator command.
                                                         If None, uses TiltAngleMapping
            g (float): Gravitational acceleration (m/s²), used for default mapping
        """
        self.Kp = Kp
        self.Kd = Kd
        self.acceleration_limit = acceleration_limit if acceleration_limit is not None else g
        
        # Default to tilt angle mapping if none provided
        if actuator_mapping is None:
            self.actuator_mapping = TiltAngleMapping(g=g)
        else:
            self.actuator_mapping = actuator_mapping
        
        # Tracking state (for diagnostics)
        self.last_error = None
        self.last_velocity_error = None
    
    def update(self, trajectory, t, x, xd):
        """Compute actuator command from trajectory and state feedback.
        
        Args:
            trajectory: Trajectory object with get_position(), get_velocity(), 
                       and get_acceleration() methods
            t (float): Current time (seconds)
            x (float): Actual position (meters)
            xd (float): Actual velocity (m/s)
            
        Returns:
            dict: {
                'actuator_command': float,  # Actuator command (units depend on mapping)
                'position_error': float,    # e = x_d - x
                'velocity_error': float,    # ed = xd_d - xd
                'commanded_acceleration': float,  # xdd_cmd
                'desired_acceleration': float,    # xdd_d
            }
        """
        # Get desired trajectory values
        x_d = trajectory.get_position(t)
        xd_d = trajectory.get_velocity(t)
        xdd_d = trajectory.get_acceleration(t)
        
        # Compute tracking errors
        e = x_d - x
        ed = xd_d - xd
        
        # Store for diagnostics
        self.last_error = e
        self.last_velocity_error = ed
        
        # Compute commanded acceleration
        # xdd_cmd = xdd_d + Kp * e + Kd * ed
        xdd_cmd = xdd_d + self.Kp * e + self.Kd * ed
        
        # Apply acceleration limit
        xdd_cmd = np.clip(xdd_cmd, -self.acceleration_limit, self.acceleration_limit)
        
        # Convert to actuator command
        actuator_command = self.actuator_mapping.acceleration_to_command(xdd_cmd)
        
        return {
            'actuator_command': actuator_command,
            'position_error': e,
            'velocity_error': ed,
            'commanded_acceleration': xdd_cmd,
            'desired_acceleration': xdd_d,
            'desired_position': x_d,
            'desired_velocity': xd_d,
        }
    
    def set_gains(self, Kp, Kd):
        """Update controller gains.
        
        Args:
            Kp (float): Proportional gain
            Kd (float): Derivative gain
        """
        self.Kp = Kp
        self.Kd = Kd
    
    def set_acceleration_limit(self, limit):
        """Update acceleration limit.
        
        Args:
            limit (float): Maximum acceleration (m/s²)
        """
        self.acceleration_limit = limit
    
    def set_actuator_mapping(self, mapping):
        """Set actuator mapping.
        
        Args:
            mapping (ActuatorMapping): Actuator mapping object
        """
        self.actuator_mapping = mapping

