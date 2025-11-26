"""
Analyze disturbance recovery metrics for all three control methods.

Metrics:
- Recovery time (time to return to within tolerance after disturbance)
- Max deviation during disturbance
- Overshoot after disturbance
- Control effort during recovery
"""

import csv
import numpy as np
from pathlib import Path

# Disturbance parameters (from experiment configuration)
DISTURBANCE_START = 5.0  # seconds
DISTURBANCE_END = 8.0   # seconds (start_time + duration)
DISTURBANCE_DURATION = 3.0  # seconds
TOLERANCE = 0.005  # 5mm in meters

# CSV files to analyze
FILES = {
    'Step PID': 'results/experiment_step_pid_20251125_213912.csv',
    'One-Shot Trajectory': 'results/experiment_oneshot_trajectory_20251125_213958.csv',
    'Trajectory Update': 'results/experiment_trajectory_update_20251125_214035.csv'
}


def load_csv(filename):
    """Load CSV file and return arrays of time, position, error, and control signal."""
    time = []
    position = []
    error = []
    control = []
    
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row['time']))
            position.append(float(row['position_x']))
            error.append(float(row['error']))
            control.append(float(row['control_signal']))
    
    return np.array(time), np.array(position), np.array(error), np.array(control)


def calculate_recovery_time(time, error, disturbance_end, tolerance):
    """
    Calculate time to return to within tolerance after disturbance ends.
    Returns time in seconds, or None if never recovers.
    """
    # Find indices after disturbance ends
    post_disturbance_mask = time >= disturbance_end
    if not np.any(post_disturbance_mask):
        return None
    
    post_disturbance_time = time[post_disturbance_mask]
    post_disturbance_error = np.abs(error[post_disturbance_mask])
    
    # Find first time when error is within tolerance
    within_tolerance = post_disturbance_error <= tolerance
    
    if not np.any(within_tolerance):
        return None  # Never recovered
    
    first_recovery_idx = np.where(within_tolerance)[0][0]
    recovery_time = post_disturbance_time[first_recovery_idx] - disturbance_end
    
    # Check if it stays within tolerance for at least 0.5 seconds
    # (settle_duration from experiment config)
    settle_duration = 0.5
    recovery_time_abs = post_disturbance_time[first_recovery_idx]
    settle_end = recovery_time_abs + settle_duration
    
    # Check if we have enough data
    if settle_end > time[-1]:
        return None  # Not enough data to confirm settling
    
    # Check if error stays within tolerance for settle_duration
    settle_mask = (time >= recovery_time_abs) & (time <= settle_end)
    if np.all(np.abs(error[settle_mask]) <= tolerance):
        return recovery_time
    
    return None  # Didn't settle properly


def calculate_max_deviation_during_disturbance(time, error, disturbance_start, disturbance_end):
    """Calculate maximum absolute error during disturbance period."""
    mask = (time >= disturbance_start) & (time <= disturbance_end)
    if not np.any(mask):
        return None
    
    disturbance_errors = np.abs(error[mask])
    return np.max(disturbance_errors)


def calculate_overshoot_after_disturbance(time, error, disturbance_end):
    """Calculate maximum absolute error after disturbance ends."""
    mask = time > disturbance_end
    if not np.any(mask):
        return None
    
    post_disturbance_errors = np.abs(error[mask])
    return np.max(post_disturbance_errors)


def calculate_control_effort_during_recovery(time, control, error, disturbance_end, tolerance):
    """
    Calculate maximum control signal magnitude during recovery period.
    Recovery period is from disturbance end until position returns to tolerance.
    """
    # Find recovery period
    post_disturbance_mask = time >= disturbance_end
    if not np.any(post_disturbance_mask):
        return None
    
    post_disturbance_time = time[post_disturbance_mask]
    post_disturbance_error = np.abs(error[post_disturbance_mask])
    post_disturbance_control = np.abs(control[post_disturbance_mask])
    
    # Find when recovery occurs
    within_tolerance = post_disturbance_error <= tolerance
    if not np.any(within_tolerance):
        # Never recovered - use all post-disturbance data
        return np.max(post_disturbance_control)
    
    first_recovery_idx = np.where(within_tolerance)[0][0]
    recovery_period_control = post_disturbance_control[:first_recovery_idx+1]
    
    if len(recovery_period_control) == 0:
        return None
    
    return np.max(recovery_period_control)


def analyze_disturbance_recovery(filename, method_name):
    """Analyze disturbance recovery metrics for a single experiment."""
    print(f"\n{'='*80}")
    print(f"Analyzing: {method_name}")
    print(f"File: {filename}")
    print(f"{'='*80}")
    
    # Load data
    time, position, error, control = load_csv(filename)
    
    # Calculate metrics
    recovery_time = calculate_recovery_time(time, error, DISTURBANCE_END, TOLERANCE)
    max_deviation = calculate_max_deviation_during_disturbance(time, error, DISTURBANCE_START, DISTURBANCE_END)
    overshoot = calculate_overshoot_after_disturbance(time, error, DISTURBANCE_END)
    control_effort = calculate_control_effort_during_recovery(time, control, error, DISTURBANCE_END, TOLERANCE)
    
    # Print results
    print(f"\nDisturbance Period: {DISTURBANCE_START:.1f}s to {DISTURBANCE_END:.1f}s")
    print(f"Tolerance: {TOLERANCE*1000:.1f} mm")
    
    print(f"\n--- Recovery Metrics ---")
    if recovery_time is not None:
        print(f"Recovery Time: {recovery_time:.3f} s")
    else:
        print(f"Recovery Time: N/A (did not recover within experiment duration)")
    
    if max_deviation is not None:
        print(f"Max Deviation During Disturbance: {max_deviation*1000:.2f} mm")
    else:
        print(f"Max Deviation During Disturbance: N/A")
    
    if overshoot is not None:
        print(f"Max Overshoot After Disturbance: {overshoot*1000:.2f} mm")
    else:
        print(f"Max Overshoot After Disturbance: N/A")
    
    if control_effort is not None:
        print(f"Max Control Effort During Recovery: {control_effort:.2f} deg")
    else:
        print(f"Max Control Effort During Recovery: N/A")
    
    return {
        'method': method_name,
        'recovery_time': recovery_time,
        'max_deviation': max_deviation,
        'overshoot': overshoot,
        'control_effort': control_effort
    }


def generate_comparison_table(results):
    """Generate a comparison table of all metrics."""
    print(f"\n{'='*80}")
    print("DISTURBANCE RECOVERY COMPARISON")
    print(f"{'='*80}")
    
    print(f"\nDisturbance: Sinusoidal actuator (2° amplitude, 1 Hz, {DISTURBANCE_DURATION}s duration)")
    print(f"Disturbance Period: {DISTURBANCE_START:.1f}s to {DISTURBANCE_END:.1f}s")
    print(f"Tolerance: {TOLERANCE*1000:.1f} mm")
    
    print(f"\n{'Method':<25} {'Recovery Time (s)':<20} {'Max Dev (mm)':<15} {'Overshoot (mm)':<15} {'Control Effort (deg)':<20}")
    print("-" * 95)
    
    for result in results:
        method = result['method']
        recovery = f"{result['recovery_time']:.3f}" if result['recovery_time'] is not None else "N/A"
        max_dev = f"{result['max_deviation']*1000:.2f}" if result['max_deviation'] is not None else "N/A"
        overshoot_val = f"{result['overshoot']*1000:.2f}" if result['overshoot'] is not None else "N/A"
        control = f"{result['control_effort']:.2f}" if result['control_effort'] is not None else "N/A"
        
        print(f"{method:<25} {recovery:<20} {max_dev:<15} {overshoot_val:<15} {control:<20}")
    
    # Summary
    print(f"\n{'='*80}")
    print("SUMMARY")
    print(f"{'='*80}")
    
    # Find best recovery time
    recovery_times = [r['recovery_time'] for r in results if r['recovery_time'] is not None]
    if recovery_times:
        best_recovery = min(recovery_times)
        best_method = [r['method'] for r in results if r['recovery_time'] == best_recovery][0]
        print(f"Fastest Recovery: {best_method} ({best_recovery:.3f} s)")
    
    # Find smallest overshoot
    overshoots = [r['overshoot'] for r in results if r['overshoot'] is not None]
    if overshoots:
        best_overshoot = min(overshoots)
        best_method = [r['method'] for r in results if r['overshoot'] == best_overshoot][0]
        print(f"Smallest Overshoot: {best_method} ({best_overshoot*1000:.2f} mm)")
    
    # Find smallest control effort
    control_efforts = [r['control_effort'] for r in results if r['control_effort'] is not None]
    if control_efforts:
        best_effort = min(control_efforts)
        best_method = [r['method'] for r in results if r['control_effort'] == best_effort][0]
        print(f"Lowest Control Effort: {best_method} ({best_effort:.2f} deg)")


def main():
    """Main analysis function."""
    print("="*80)
    print("DISTURBANCE RECOVERY ANALYSIS")
    print("="*80)
    
    results = []
    
    for method_name, filename in FILES.items():
        filepath = Path(filename)
        if not filepath.exists():
            print(f"\nWARNING: File not found: {filename}")
            continue
        
        result = analyze_disturbance_recovery(filename, method_name)
        results.append(result)
    
    if results:
        generate_comparison_table(results)
    
    print(f"\n{'='*80}")
    print("Analysis complete!")
    print(f"{'='*80}")


if __name__ == "__main__":
    main()

