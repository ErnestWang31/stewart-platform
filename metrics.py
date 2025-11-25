# Metrics Module
# Computes performance metrics for controller comparison

import numpy as np

def compute_completion_time(x_data, t_data, tolerance=0.005, settle_duration=0.5):
    """Compute completion time: first time |x| < tolerance for ≥ settle_duration seconds.
    
    Args:
        x_data (np.array): Position data (meters)
        t_data (np.array): Time data (seconds)
        tolerance (float): Position tolerance (meters)
        settle_duration (float): Required settle duration (seconds)
        
    Returns:
        float: Completion time in seconds, or None if never completed
    """
    if len(x_data) == 0 or len(t_data) == 0:
        return None
    
    if len(x_data) != len(t_data):
        raise ValueError("x_data and t_data must have same length")
    
    # Find all times when |x| < tolerance
    within_tolerance = np.abs(x_data) < tolerance
    
    if not np.any(within_tolerance):
        return None
    
    # Find continuous periods where |x| < tolerance
    # Check if there's a period of at least settle_duration
    dt = t_data[1] - t_data[0] if len(t_data) > 1 else 0.01
    min_samples = int(settle_duration / dt)
    
    if min_samples < 1:
        min_samples = 1
    
    # Find first occurrence of settle_duration consecutive samples within tolerance
    for i in range(len(within_tolerance) - min_samples + 1):
        if np.all(within_tolerance[i:i+min_samples]):
            # Return the time at the end of the settle period
            return t_data[i + min_samples - 1]
    
    return None


def compute_rmse(e_data):
    """Compute root mean square error.
    
    Args:
        e_data (np.array): Error data (meters)
        
    Returns:
        float: RMSE in meters
    """
    if len(e_data) == 0:
        return 0.0
    
    return np.sqrt(np.mean(e_data ** 2))


def compute_max_deviation(e_data):
    """Compute maximum deviation (overshoot).
    
    Args:
        e_data (np.array): Error data (meters)
        
    Returns:
        float: Maximum absolute error (meters)
    """
    if len(e_data) == 0:
        return 0.0
    
    return np.max(np.abs(e_data))


def compute_control_effort(u_data, u_limit=15.0):
    """Compute control effort metrics.
    
    Args:
        u_data (np.array): Control signal data (degrees)
        u_limit (float): Control signal limit (degrees)
        
    Returns:
        dict: Dictionary with:
            - 'max_control': Maximum absolute control signal
            - 'saturation_percent': Percentage of samples that hit saturation
    """
    if len(u_data) == 0:
        return {'max_control': 0.0, 'saturation_percent': 0.0}
    
    max_control = np.max(np.abs(u_data))
    
    # Count saturation events
    saturated = np.abs(u_data) >= u_limit
    saturation_percent = 100.0 * np.sum(saturated) / len(u_data)
    
    return {
        'max_control': max_control,
        'saturation_percent': saturation_percent
    }


def compute_smoothness(x_data, t_data):
    """Compute smoothness metric using jerk (derivative of acceleration).
    
    Args:
        x_data (np.array): Position data (meters)
        t_data (np.array): Time data (seconds)
        
    Returns:
        dict: Dictionary with:
            - 'max_jerk': Maximum absolute jerk (m/s^3)
            - 'rms_jerk': RMS jerk (m/s^3)
            - 'velocity_reversals': Number of velocity direction changes
    """
    if len(x_data) < 3 or len(t_data) < 3:
        return {'max_jerk': 0.0, 'rms_jerk': 0.0, 'velocity_reversals': 0}
    
    if len(x_data) != len(t_data):
        raise ValueError("x_data and t_data must have same length")
    
    # Compute velocity using finite differences
    dt = np.diff(t_data)
    # Avoid division by zero
    dt = np.where(dt > 1e-6, dt, 1e-6)
    velocity = np.diff(x_data) / dt
    
    # Compute acceleration
    if len(velocity) < 2:
        return {'max_jerk': 0.0, 'rms_jerk': 0.0, 'velocity_reversals': 0}
    
    dt_accel = dt[:-1]
    dt_accel = np.where(dt_accel > 1e-6, dt_accel, 1e-6)
    acceleration = np.diff(velocity) / dt_accel
    
    # Compute jerk
    if len(acceleration) < 2:
        return {'max_jerk': 0.0, 'rms_jerk': 0.0, 'velocity_reversals': 0}
    
    dt_jerk = dt_accel[:-1]
    dt_jerk = np.where(dt_jerk > 1e-6, dt_jerk, 1e-6)
    jerk = np.diff(acceleration) / dt_jerk
    
    max_jerk = np.max(np.abs(jerk)) if len(jerk) > 0 else 0.0
    rms_jerk = np.sqrt(np.mean(jerk ** 2)) if len(jerk) > 0 else 0.0
    
    # Count velocity reversals (sign changes in velocity)
    if len(velocity) < 2:
        velocity_reversals = 0
    else:
        velocity_sign_changes = np.diff(np.sign(velocity))
        velocity_reversals = np.sum(np.abs(velocity_sign_changes) > 0)
    
    return {
        'max_jerk': max_jerk,
        'rms_jerk': rms_jerk,
        'velocity_reversals': velocity_reversals
    }


def compute_all_metrics(x_data, t_data, e_data, u_data, tolerance=0.005, settle_duration=0.5, u_limit=15.0):
    """Compute all metrics at once.
    
    Args:
        x_data (np.array): Position data (meters)
        t_data (np.array): Time data (seconds)
        e_data (np.array): Error data (meters)
        u_data (np.array): Control signal data (degrees)
        tolerance (float): Position tolerance for completion (meters)
        settle_duration (float): Required settle duration (seconds)
        u_limit (float): Control signal limit (degrees)
        
    Returns:
        dict: Dictionary with all computed metrics
    """
    completion_time = compute_completion_time(x_data, t_data, tolerance, settle_duration)
    rmse = compute_rmse(e_data)
    max_deviation = compute_max_deviation(e_data)
    control_effort = compute_control_effort(u_data, u_limit)
    smoothness = compute_smoothness(x_data, t_data)
    
    return {
        'completion_time': completion_time,
        'rmse': rmse,
        'max_deviation': max_deviation,
        'max_control': control_effort['max_control'],
        'saturation_percent': control_effort['saturation_percent'],
        'max_jerk': smoothness['max_jerk'],
        'rms_jerk': smoothness['rms_jerk'],
        'velocity_reversals': smoothness['velocity_reversals']
    }

