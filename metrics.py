# Metrics Calculation Module
# Computes performance metrics from experiment data

import numpy as np

def completion_time(time_array, position_array, tolerance=0.005, settle_duration=0.5):
    """Calculate completion time: first time |x(t)| < tolerance for ≥ settle_duration seconds.
    
    Args:
        time_array: Array of time values (seconds)
        position_array: Array of position values (meters)
        tolerance: Position tolerance in meters (default 5mm)
        settle_duration: Required duration within tolerance (default 0.5s)
        
    Returns:
        completion_time: Time in seconds, or None if never completed
    """
    if len(time_array) == 0 or len(position_array) == 0:
        return None
    
    time_array = np.array(time_array)
    position_array = np.array(position_array)
    
    # Find all times when within tolerance
    within_tolerance = np.abs(position_array) < tolerance
    
    if not np.any(within_tolerance):
        return None
    
    # Find continuous periods within tolerance
    settle_start = None
    for i in range(len(time_array)):
        if within_tolerance[i]:
            if settle_start is None:
                settle_start = time_array[i]
            # Check if we've been in tolerance long enough
            if time_array[i] - settle_start >= settle_duration:
                return settle_start
        else:
            settle_start = None
    
    return None

def rmse_error(error_array):
    """Calculate Root Mean Square Error of tracking error.
    
    Args:
        error_array: Array of error values e(t) = r(t) - x(t)
        
    Returns:
        rmse: Root mean square error
    """
    if len(error_array) == 0:
        return 0.0
    return np.sqrt(np.mean(np.array(error_array)**2))

def max_deviation(error_array):
    """Calculate maximum absolute deviation from setpoint.
    
    Args:
        error_array: Array of error values e(t) = r(t) - x(t)
        
    Returns:
        max_dev: Maximum absolute error
    """
    if len(error_array) == 0:
        return 0.0
    return np.max(np.abs(np.array(error_array)))

def max_control_effort(control_array):
    """Calculate maximum absolute control effort.
    
    Args:
        control_array: Array of control signal values u(t)
        
    Returns:
        max_control: Maximum absolute control signal
    """
    if len(control_array) == 0:
        return 0.0
    return np.max(np.abs(np.array(control_array)))

def saturation_percentage(saturation_array):
    """Calculate percentage of time actuators were saturated.
    
    Args:
        saturation_array: Array of saturation flags (1 if saturated, 0 otherwise)
        
    Returns:
        saturation_pct: Percentage of samples with saturation (0-100)
    """
    if len(saturation_array) == 0:
        return 0.0
    saturation_array = np.array(saturation_array)
    return 100.0 * np.sum(saturation_array) / len(saturation_array)

def jerk_metrics(time_array, position_array):
    """Calculate jerk metrics (derivative of acceleration).
    
    Args:
        time_array: Array of time values (seconds)
        position_array: Array of position values (meters)
        
    Returns:
        max_jerk: Maximum absolute jerk (m/s³)
        rms_jerk: Root mean square jerk (m/s³)
    """
    if len(time_array) < 3 or len(position_array) < 3:
        return 0.0, 0.0
    
    time_array = np.array(time_array)
    position_array = np.array(position_array)
    
    # Calculate velocity (first derivative)
    dt = np.diff(time_array)
    # Avoid division by zero
    dt = np.where(dt > 1e-6, dt, 1e-6)
    velocity = np.diff(position_array) / dt
    
    if len(velocity) < 2:
        return 0.0, 0.0
    
    # Calculate acceleration (second derivative)
    dt_vel = np.diff(time_array[1:])
    dt_vel = np.where(dt_vel > 1e-6, dt_vel, 1e-6)
    acceleration = np.diff(velocity) / dt_vel
    
    if len(acceleration) < 1:
        return 0.0, 0.0
    
    # Calculate jerk (third derivative)
    dt_acc = np.diff(time_array[2:])
    dt_acc = np.where(dt_acc > 1e-6, dt_acc, 1e-6)
    jerk = np.diff(acceleration) / dt_acc
    
    if len(jerk) == 0:
        return 0.0, 0.0
    
    max_jerk = np.max(np.abs(jerk))
    rms_jerk = np.sqrt(np.mean(jerk**2))
    
    return max_jerk, rms_jerk

def velocity_reversals(time_array, position_array):
    """Count number of velocity direction reversals (smoothness metric).
    
    Args:
        time_array: Array of time values (seconds)
        position_array: Array of position values (meters)
        
    Returns:
        reversals: Number of times velocity changes sign
    """
    if len(time_array) < 2 or len(position_array) < 2:
        return 0
    
    time_array = np.array(time_array)
    position_array = np.array(position_array)
    
    # Calculate velocity
    dt = np.diff(time_array)
    dt = np.where(dt > 1e-6, dt, 1e-6)
    velocity = np.diff(position_array) / dt
    
    if len(velocity) < 2:
        return 0
    
    # Count sign changes
    sign_changes = np.diff(np.sign(velocity))
    reversals = np.sum(np.abs(sign_changes) > 0)
    
    return reversals

def compute_all_metrics(time_array, position_array, setpoint_array, error_array, 
                       control_array, tilt_array, saturation_array,
                       tolerance=0.005, settle_duration=0.5):
    """Compute all metrics from experiment data.
    
    Args:
        time_array: Array of time values
        position_array: Array of measured positions
        setpoint_array: Array of desired positions
        error_array: Array of errors (r - x)
        control_array: Array of control signals
        tilt_array: Array of tilt angles
        saturation_array: Array of saturation flags
        tolerance: Completion tolerance (meters)
        settle_duration: Required settle duration (seconds)
        
    Returns:
        dict: Dictionary with all computed metrics
    """
    metrics = {
        'completion_time': completion_time(time_array, position_array, tolerance, settle_duration),
        'rmse': rmse_error(error_array),
        'max_deviation': max_deviation(error_array),
        'max_control': max_control_effort(control_array),
        'saturation_pct': saturation_percentage(saturation_array),
    }
    
    max_jerk, rms_jerk = jerk_metrics(time_array, position_array)
    metrics['max_jerk'] = max_jerk
    metrics['rms_jerk'] = rms_jerk
    metrics['velocity_reversals'] = velocity_reversals(time_array, position_array)
    
    return metrics

def print_metrics(metrics, method_name="Experiment"):
    """Print metrics in a formatted way.
    
    Args:
        metrics: Dictionary of metrics from compute_all_metrics
        method_name: Name of the control method
    """
    print(f"\n=== {method_name} Metrics ===")
    print(f"Completion Time: {metrics['completion_time']:.3f}s" if metrics['completion_time'] is not None else "Completion Time: N/A")
    print(f"RMSE: {metrics['rmse']:.6f} m")
    print(f"Max Deviation: {metrics['max_deviation']:.6f} m")
    print(f"Max Control: {metrics['max_control']:.3f}°")
    print(f"Saturation: {metrics['saturation_pct']:.1f}%")
    print(f"Max Jerk: {metrics['max_jerk']:.3f} m/s³")
    print(f"RMS Jerk: {metrics['rms_jerk']:.3f} m/s³")
    print(f"Velocity Reversals: {metrics['velocity_reversals']}")

