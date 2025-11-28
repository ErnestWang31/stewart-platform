# Trajectory Generator Module
# Generates smooth trajectories from initial to final position

import numpy as np

class Trajectory:
    """Represents a trajectory from x0 to xf over duration T."""
    
    def __init__(self, x0, xf, T, method='linear', curvature=3.0):
        """Initialize trajectory.
        
        Args:
            x0: Initial position (meters)
            xf: Final position (meters)
            T: Duration (seconds)
            method: 'linear', 'polynomial', or 'exponential' (default: 'linear')
            curvature: Curvature parameter for exponential method (default: 3.0, higher = more curved)
        """
        self.x0 = x0
        self.xf = xf
        self.T = T
        self.method = method
        self.curvature = curvature  # For exponential trajectory
        
        if method == 'linear':
            # Linear trajectory: r(t) = x0 + (xf - x0) * (t/T)
            # Velocity: v(t) = (xf - x0) / T (constant)
            self.velocity = (xf - x0) / T if T > 0 else 0.0
        elif method == 'polynomial':
            # 5th order polynomial for smooth trajectory
            # r(0) = x0, r(T) = xf
            # v(0) = 0, v(T) = 0 (zero velocity at endpoints)
            # a(0) = 0, a(T) = 0 (zero acceleration at endpoints)
            # This will be computed when needed
            pass
        elif method == 'exponential':
            # Exponential decay trajectory: starts fast, slows down
            # r(t) = x0 + (xf - x0) * (1 - e^(-k*t/T)) / (1 - e^(-k))
            # where k is the curvature parameter
            # This ensures r(0) = x0 and r(T) = xf exactly
            pass
        else:
            raise ValueError(f"Unknown trajectory method: {method}. Use 'linear', 'polynomial', or 'exponential'")
    
    def get_position(self, t):
        """Get position at time t.
        
        Args:
            t: Time in seconds (0 <= t <= T)
            
        Returns:
            position: Position at time t
        """
        t = np.clip(t, 0.0, self.T)
        
        if self.method == 'linear':
            # Linear interpolation
            if self.T == 0:
                return self.xf
            return self.x0 + (self.xf - self.x0) * (t / self.T)
        
        elif self.method == 'polynomial':
            # 5th order polynomial trajectory
            # r(t) = x0 + (xf - x0) * (10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5)
            if self.T == 0:
                return self.xf
            tau = t / self.T  # Normalized time [0, 1]
            # 5th order polynomial with zero velocity and acceleration at endpoints
            # This is a smooth S-curve
            poly = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            return self.x0 + (self.xf - self.x0) * poly
        
        elif self.method == 'exponential':
            # Exponential decay trajectory: starts fast, slows down
            # r(t) = x0 + (xf - x0) * (1 - e^(-k*t/T)) / (1 - e^(-k))
            # where k is the curvature parameter
            if self.T == 0:
                return self.xf
            tau = t / self.T  # Normalized time [0, 1]
            k = self.curvature
            # Normalized exponential decay
            # At tau=0: exp_term = 0, so result = x0
            # At tau=1: exp_term = 1 - e^(-k), normalized by (1 - e^(-k)), so result = xf
            exp_term = (1 - np.exp(-k * tau)) / (1 - np.exp(-k)) if k > 0 else tau
            return self.x0 + (self.xf - self.x0) * exp_term
        
        return self.x0
    
    def get_velocity(self, t):
        """Get velocity at time t.
        
        Args:
            t: Time in seconds (0 <= t <= T)
            
        Returns:
            velocity: Velocity at time t (m/s)
        """
        t = np.clip(t, 0.0, self.T)
        
        if self.method == 'linear':
            return self.velocity
        
        elif self.method == 'polynomial':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            # Derivative of 5th order polynomial
            # v(t) = (xf - x0) / T * (30*tau^2 - 60*tau^3 + 30*tau^4)
            dtau = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
            return (self.xf - self.x0) / self.T * dtau
        
        elif self.method == 'exponential':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            k = self.curvature
            # Derivative of exponential: v(t) = (xf - x0) / T * k * e^(-k*tau) / (1 - e^(-k))
            if k > 0:
                dtau = k * np.exp(-k * tau) / (1 - np.exp(-k))
            else:
                dtau = 1.0
            return (self.xf - self.x0) / self.T * dtau
        
        return 0.0
    
    def get_acceleration(self, t):
        """Get acceleration at time t.
        
        Args:
            t: Time in seconds (0 <= t <= T)
            
        Returns:
            acceleration: Acceleration at time t (m/s²)
        """
        t = np.clip(t, 0.0, self.T)
        
        if self.method == 'linear':
            # Linear trajectory has zero acceleration (constant velocity)
            return 0.0
        
        elif self.method == 'polynomial':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            # Second derivative of 5th order polynomial
            # a(t) = (xf - x0) / T² * (60*tau - 180*tau^2 + 120*tau^3)
            ddtau = 60 * tau - 180 * tau**2 + 120 * tau**3
            return (self.xf - self.x0) / (self.T**2) * ddtau
        
        elif self.method == 'exponential':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            k = self.curvature
            # Second derivative of exponential: a(t) = (xf - x0) / T² * k² * e^(-k*tau) / (1 - e^(-k))
            if k > 0:
                ddtau = -k**2 * np.exp(-k * tau) / (1 - np.exp(-k))
            else:
                ddtau = 0.0
            return (self.xf - self.x0) / (self.T**2) * ddtau
        
        return 0.0
    
    def is_complete(self, t):
        """Check if trajectory is complete at time t.
        
        Args:
            t: Time in seconds
            
        Returns:
            bool: True if t >= T
        """
        return t >= self.T


def generate_trajectory(x0, xf, T, method='linear', curvature=3.0):
    """Generate a trajectory from x0 to xf over duration T.
    
    Args:
        x0: Initial position (meters)
        xf: Final position (meters)
        T: Duration (seconds)
        method: 'linear', 'polynomial', or 'exponential' (default: 'linear')
        curvature: Curvature parameter for exponential method (default: 3.0, higher = more curved)
        
    Returns:
        Trajectory: Trajectory object
    """
    return Trajectory(x0, xf, T, method, curvature)


def sample_trajectory(trajectory, t):
    """Sample position from trajectory at time t.
    
    Args:
        trajectory: Trajectory object
        t: Time in seconds
        
    Returns:
        position: Position at time t
    """
    return trajectory.get_position(t)

