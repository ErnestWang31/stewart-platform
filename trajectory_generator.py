# Trajectory Generator Module
# Generates smooth trajectories for ball control from initial to final position

import numpy as np

def generate_trajectory(x0, xf, T, method='polynomial', order=5):
    """Generate a trajectory from x0 to xf over time T.
    
    Args:
        x0 (float): Initial position (meters)
        xf (float): Final position (meters)
        T (float): Time duration (seconds)
        method (str): 'polynomial' or 'linear'
        order (int): Polynomial order (3 or 5, only used for polynomial method)
        
    Returns:
        dict: Trajectory parameters containing:
            - 'method': trajectory method
            - 'x0': initial position
            - 'xf': final position
            - 'T': time duration
            - 'coeffs': polynomial coefficients (if polynomial)
            - 'order': polynomial order (if polynomial)
    """
    if method == 'linear':
        return {
            'method': 'linear',
            'x0': x0,
            'xf': xf,
            'T': T
        }
    elif method == 'polynomial':
        if order == 5:
            # 5th-order polynomial: x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            # Boundary conditions:
            # x(0) = x0, x(T) = xf
            # x_dot(0) = 0, x_dot(T) = 0
            # x_ddot(0) = 0, x_ddot(T) = 0
            
            # Solve for coefficients
            # x(0) = a0 = x0
            # x(T) = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5*T^5 = xf
            # x_dot(0) = a1 = 0
            # x_dot(T) = a1 + 2*a2*T + 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4 = 0
            # x_ddot(0) = 2*a2 = 0
            # x_ddot(T) = 2*a2 + 6*a3*T + 12*a4*T^2 + 20*a5*T^3 = 0
            
            a0 = x0
            a1 = 0.0
            a2 = 0.0
            
            # Solve system for a3, a4, a5
            # xf - x0 = a3*T^3 + a4*T^4 + a5*T^5
            # 0 = 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4
            # 0 = 6*a3*T + 12*a4*T^2 + 20*a5*T^3
            
            # From third equation: a3 = -(2*a4*T + 10/3*a5*T^2)
            # From second equation: 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4 = 0
            # Substitute and solve...
            
            # Direct solution:
            delta_x = xf - x0
            T2 = T * T
            T3 = T2 * T
            T4 = T3 * T
            T5 = T4 * T
            
            a3 = 10 * delta_x / T3
            a4 = -15 * delta_x / T4
            a5 = 6 * delta_x / T5
            
            coeffs = np.array([a0, a1, a2, a3, a4, a5])
            
        elif order == 3:
            # 3rd-order polynomial: x(t) = a0 + a1*t + a2*t^2 + a3*t^3
            # Boundary conditions:
            # x(0) = x0, x(T) = xf
            # x_dot(0) = 0, x_dot(T) = 0
            
            a0 = x0
            a1 = 0.0
            
            delta_x = xf - x0
            T2 = T * T
            T3 = T2 * T
            
            a2 = 3 * delta_x / T2
            a3 = -2 * delta_x / T3
            
            coeffs = np.array([a0, a1, a2, a3])
        else:
            raise ValueError(f"Polynomial order must be 3 or 5, got {order}")
        
        return {
            'method': 'polynomial',
            'x0': x0,
            'xf': xf,
            'T': T,
            'coeffs': coeffs,
            'order': order
        }
    else:
        raise ValueError(f"Unknown trajectory method: {method}")


def sample_trajectory(traj, t):
    """Sample trajectory at time t.
    
    Args:
        traj (dict): Trajectory parameters from generate_trajectory()
        t (float): Time at which to sample (seconds)
        
    Returns:
        float: Position at time t (meters)
    """
    if t < 0:
        return traj['x0']
    
    if traj['method'] == 'linear':
        T = traj['T']
        if t >= T:
            return traj['xf']
        # Linear interpolation
        alpha = t / T
        return traj['x0'] + alpha * (traj['xf'] - traj['x0'])
    
    elif traj['method'] == 'polynomial':
        T = traj['T']
        if t >= T:
            return traj['xf']
        
        coeffs = traj['coeffs']
        order = traj['order']
        
        # Evaluate polynomial: x(t) = sum(coeffs[i] * t^i)
        position = 0.0
        for i, coeff in enumerate(coeffs):
            position += coeff * (t ** i)
        
        return position
    
    else:
        raise ValueError(f"Unknown trajectory method: {traj['method']}")


def get_trajectory_velocity(traj, t):
    """Get velocity at time t (first derivative).
    
    Args:
        traj (dict): Trajectory parameters
        t (float): Time at which to sample (seconds)
        
    Returns:
        float: Velocity at time t (m/s)
    """
    if t < 0 or t >= traj['T']:
        return 0.0
    
    if traj['method'] == 'linear':
        return (traj['xf'] - traj['x0']) / traj['T']
    
    elif traj['method'] == 'polynomial':
        coeffs = traj['coeffs']
        # Derivative: x_dot(t) = sum(i * coeffs[i] * t^(i-1)) for i >= 1
        velocity = 0.0
        for i in range(1, len(coeffs)):
            velocity += i * coeffs[i] * (t ** (i - 1))
        return velocity
    
    else:
        raise ValueError(f"Unknown trajectory method: {traj['method']}")


def get_trajectory_acceleration(traj, t):
    """Get acceleration at time t (second derivative).
    
    Args:
        traj (dict): Trajectory parameters
        t (float): Time at which to sample (seconds)
        
    Returns:
        float: Acceleration at time t (m/s^2)
    """
    if t < 0 or t >= traj['T']:
        return 0.0
    
    if traj['method'] == 'linear':
        return 0.0
    
    elif traj['method'] == 'polynomial':
        coeffs = traj['coeffs']
        # Second derivative: x_ddot(t) = sum(i*(i-1) * coeffs[i] * t^(i-2)) for i >= 2
        acceleration = 0.0
        for i in range(2, len(coeffs)):
            acceleration += i * (i - 1) * coeffs[i] * (t ** (i - 2))
        return acceleration
    
    else:
        raise ValueError(f"Unknown trajectory method: {traj['method']}")

