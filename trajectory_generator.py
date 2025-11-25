# Trajectory Generator Module
# Generates smooth trajectories from initial to final position

import numpy as np

class Trajectory:
    """Represents a trajectory from x0 to xf over duration T."""
    
    def __init__(self, x0, xf, T, method='linear'):
        """Initialize trajectory.
        
        Args:
            x0: Initial position (meters)
            xf: Final position (meters)
            T: Duration (seconds)
            method: 'linear' or 'polynomial' (default: 'linear')
        """
        self.x0 = x0
        self.xf = xf
        self.T = T
        self.method = method
        
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
        else:
            raise ValueError(f"Unknown trajectory method: {method}")
    
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
        
        return 0.0
    
    def is_complete(self, t):
        """Check if trajectory is complete at time t.
        
        Args:
            t: Time in seconds
            
        Returns:
            bool: True if t >= T
        """
        return t >= self.T


def generate_trajectory(x0, xf, T, method='linear'):
    """Generate a trajectory from x0 to xf over duration T.
    
    Args:
        x0: Initial position (meters)
        xf: Final position (meters)
        T: Duration (seconds)
        method: 'linear' or 'polynomial' (default: 'linear')
        
    Returns:
        Trajectory: Trajectory object
    """
    return Trajectory(x0, xf, T, method)


def sample_trajectory(trajectory, t):
    """Sample position from trajectory at time t.
    
    Args:
        trajectory: Trajectory object
        t: Time in seconds
        
    Returns:
        position: Position at time t
    """
    return trajectory.get_position(t)

