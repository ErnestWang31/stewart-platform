# Trajectory Generator Module
# Generates smooth trajectories from initial to final position

import numpy as np

class Trajectory:
    """Represents a trajectory from x0 to xf over duration T."""
    
    def __init__(self, x0, xf, T, method='linear', curvature=3.0, v0=0.0, vf=0.0):
        """Initialize trajectory.
        
        Args:
            x0: Initial position (meters)
            xf: Final position (meters)
            T: Duration (seconds)
            method: 'linear', 'polynomial', 'exponential', or 'min_jerk' (default: 'linear')
            curvature: Curvature parameter for exponential method (default: 3.0, higher = more curved)
            v0: Initial velocity (m/s, default: 0.0)
            vf: Final velocity (m/s, default: 0.0)
        """
        self.x0 = x0
        self.xf = xf
        self.T = T
        self.method = method
        self.curvature = curvature  # For exponential trajectory
        self.v0 = v0  # Initial velocity
        self.vf = vf  # Final velocity
        
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
        elif method == 'min_jerk':
            # Minimum-jerk, acceleration-based trajectory
            # 7th order polynomial with C² continuity
            # r(0) = x0, r(T) = xf
            # v(0) = 0, v(T) = 0 (zero velocity at endpoints)
            # a(0) = 0, a(T) = 0 (zero acceleration at endpoints)
            # j(0) = 0, j(T) = 0 (zero jerk at endpoints)
            # This will be computed when needed
            pass
        else:
            raise ValueError(f"Unknown trajectory method: {method}. Use 'linear', 'polynomial', 'exponential', or 'min_jerk'")
    
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
            # 5th order polynomial trajectory with velocity constraints
            # r(0) = x0, r(T) = xf
            # v(0) = v0, v(T) = vf
            # a(0) = 0, a(T) = 0 (zero acceleration at endpoints)
            if self.T == 0:
                return self.xf
            tau = t / self.T  # Normalized time [0, 1]
            
            # If initial/final velocities are zero, use standard polynomial
            if abs(self.v0) < 1e-6 and abs(self.vf) < 1e-6:
                poly = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
                return self.x0 + (self.xf - self.x0) * poly
            else:
                # 5th order polynomial with velocity constraints
                # Coefficients solved to satisfy: r(0)=x0, r(T)=xf, v(0)=v0, v(T)=vf, a(0)=0, a(T)=0
                # r(tau) = x0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5
                # Solving the constraints gives:
                dx = self.xf - self.x0
                v0_norm = self.v0 * self.T  # Normalized initial velocity
                vf_norm = self.vf * self.T  # Normalized final velocity
                
                c1 = v0_norm
                c2 = 0.0  # a(0) = 0
                c3 = 10*dx - 4*v0_norm - 6*vf_norm
                c4 = -15*dx + 7*v0_norm + 8*vf_norm
                c5 = 6*dx - 3*v0_norm - 3*vf_norm
                
                return self.x0 + c1*tau + c2*tau**2 + c3*tau**3 + c4*tau**4 + c5*tau**5
        
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
        
        elif self.method == 'min_jerk':
            # Minimum-jerk trajectory: 7th order polynomial
            # C² continuous with zero end jerk
            # r(0) = x0, r(T) = xf
            # v(0) = 0, v(T) = 0
            # a(0) = 0, a(T) = 0
            # j(0) = 0, j(T) = 0
            if self.T == 0:
                return self.xf
            tau = t / self.T  # Normalized time [0, 1]
            dx = self.xf - self.x0
            
            # 7th order polynomial: r(tau) = x0 + c4*tau^4 + c5*tau^5 + c6*tau^6 + c7*tau^7
            # Solving constraints: r(1)=xf, v(1)=0, a(1)=0, j(1)=0
            # This gives: c4 = 35*dx, c5 = -84*dx, c6 = 70*dx, c7 = -20*dx
            poly = 35 * tau**4 - 84 * tau**5 + 70 * tau**6 - 20 * tau**7
            return self.x0 + dx * poly
        
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
            
            # If initial/final velocities are zero, use standard polynomial
            if abs(self.v0) < 1e-6 and abs(self.vf) < 1e-6:
                # Derivative of 5th order polynomial
                # v(t) = (xf - x0) / T * (30*tau^2 - 60*tau^3 + 30*tau^4)
                dtau = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
                return (self.xf - self.x0) / self.T * dtau
            else:
                # Velocity with velocity constraints
                dx = self.xf - self.x0
                v0_norm = self.v0 * self.T
                vf_norm = self.vf * self.T
                
                c1 = v0_norm
                c2 = 0.0
                c3 = 10*dx - 4*v0_norm - 6*vf_norm
                c4 = -15*dx + 7*v0_norm + 8*vf_norm
                c5 = 6*dx - 3*v0_norm - 3*vf_norm
                
                # Derivative: v = (1/T) * (c1 + 2*c2*tau + 3*c3*tau^2 + 4*c4*tau^3 + 5*c5*tau^4)
                return (c1 + 2*c2*tau + 3*c3*tau**2 + 4*c4*tau**3 + 5*c5*tau**4) / self.T
        
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
        
        elif self.method == 'min_jerk':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            dx = self.xf - self.x0
            # Derivative of 7th order polynomial: v(tau) = 140*tau^3 - 420*tau^4 + 420*tau^5 - 140*tau^6
            dtau = 140 * tau**3 - 420 * tau**4 + 420 * tau**5 - 140 * tau**6
            return dx / self.T * dtau
        
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
            
            # If initial/final velocities are zero, use standard polynomial
            if abs(self.v0) < 1e-6 and abs(self.vf) < 1e-6:
                # Second derivative of 5th order polynomial
                # a(t) = (xf - x0) / T² * (60*tau - 180*tau^2 + 120*tau^3)
                ddtau = 60 * tau - 180 * tau**2 + 120 * tau**3
                return (self.xf - self.x0) / (self.T ** 2) * ddtau
            else:
                # Acceleration with velocity constraints
                dx = self.xf - self.x0
                v0_norm = self.v0 * self.T
                vf_norm = self.vf * self.T
                
                c1 = v0_norm
                c2 = 0.0
                c3 = 10*dx - 4*v0_norm - 6*vf_norm
                c4 = -15*dx + 7*v0_norm + 8*vf_norm
                c5 = 6*dx - 3*v0_norm - 3*vf_norm
                
                # Second derivative: a = (1/T²) * (2*c2 + 6*c3*tau + 12*c4*tau^2 + 20*c5*tau^3)
                return (2*c2 + 6*c3*tau + 12*c4*tau**2 + 20*c5*tau**3) / (self.T ** 2)
        
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
        
        elif self.method == 'min_jerk':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            dx = self.xf - self.x0
            # Second derivative of 7th order polynomial: a(tau) = 420*tau^2 - 1680*tau^3 + 2100*tau^4 - 840*tau^5
            ddtau = 420 * tau**2 - 1680 * tau**3 + 2100 * tau**4 - 840 * tau**5
            return dx / (self.T**2) * ddtau
        
        return 0.0
    
    def get_jerk(self, t):
        """Get jerk (third derivative of position) at time t.
        
        Args:
            t: Time in seconds (0 <= t <= T)
            
        Returns:
            jerk: Jerk at time t (m/s³)
        """
        t = np.clip(t, 0.0, self.T)
        
        if self.method == 'linear':
            # Linear trajectory has zero jerk (constant velocity, zero acceleration)
            return 0.0
        
        elif self.method == 'polynomial':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            
            # If initial/final velocities are zero, use standard polynomial
            if abs(self.v0) < 1e-6 and abs(self.vf) < 1e-6:
                # Third derivative of 5th order polynomial
                # j(t) = (xf - x0) / T³ * (60 - 360*tau + 360*tau^2)
                dddtau = 60 - 360 * tau + 360 * tau**2
                return (self.xf - self.x0) / (self.T ** 3) * dddtau
            else:
                # Jerk with velocity constraints
                dx = self.xf - self.x0
                v0_norm = self.v0 * self.T
                vf_norm = self.vf * self.T
                
                c3 = 10*dx - 4*v0_norm - 6*vf_norm
                c4 = -15*dx + 7*v0_norm + 8*vf_norm
                c5 = 6*dx - 3*v0_norm - 3*vf_norm
                
                # Third derivative: j = (1/T³) * (6*c3 + 24*c4*tau + 60*c5*tau^2)
                return (6*c3 + 24*c4*tau + 60*c5*tau**2) / (self.T ** 3)
        
        elif self.method == 'exponential':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            k = self.curvature
            # Third derivative of exponential: j(t) = (xf - x0) / T³ * k³ * e^(-k*tau) / (1 - e^(-k))
            if k > 0:
                dddtau = k**3 * np.exp(-k * tau) / (1 - np.exp(-k))
            else:
                dddtau = 0.0
            return (self.xf - self.x0) / (self.T**3) * dddtau
        
        elif self.method == 'min_jerk':
            if self.T == 0:
                return 0.0
            tau = t / self.T
            dx = self.xf - self.x0
            # Third derivative of 7th order polynomial: j(tau) = 840*tau - 5040*tau^2 + 8400*tau^3 - 4200*tau^4
            # At tau=0: j(0) = 0 ✓
            # At tau=1: j(1) = 840 - 5040 + 8400 - 4200 = 0 ✓
            dddtau = 840 * tau - 5040 * tau**2 + 8400 * tau**3 - 4200 * tau**4
            return dx / (self.T**3) * dddtau
        
        return 0.0
    
    def is_complete(self, t):
        """Check if trajectory is complete at time t.
        
        Args:
            t: Time in seconds
            
        Returns:
            bool: True if t >= T
        """
        return t >= self.T


def generate_trajectory(x0, xf, T, method='linear', curvature=3.0, v0=0.0, vf=0.0):
    """Generate a trajectory from x0 to xf over duration T.
    
    Args:
        x0: Initial position (meters)
        xf: Final position (meters)
        T: Duration (seconds)
        method: 'linear', 'polynomial', 'exponential', or 'min_jerk' (default: 'linear')
        curvature: Curvature parameter for exponential method (default: 3.0, higher = more curved)
        v0: Initial velocity (m/s, default: 0.0)
        vf: Final velocity (m/s, default: 0.0)
        
    Returns:
        Trajectory: Trajectory object
    """
    return Trajectory(x0, xf, T, method, curvature, v0, vf)


def sample_trajectory(trajectory, t):
    """Sample position from trajectory at time t.
    
    Args:
        trajectory: Trajectory object
        t: Time in seconds
        
    Returns:
        position: Position at time t
    """
    return trajectory.get_position(t)

