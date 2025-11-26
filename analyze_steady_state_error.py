# Analyze steady-state error during disturbance period
import csv
import numpy as np
from pathlib import Path

# Find latest files for each experiment type
results_dir = Path("results")

step_pid_files = list(results_dir.glob("experiment_step_pid_*.csv"))
oneshot_files = list(results_dir.glob("experiment_oneshot_trajectory_*.csv"))
trajectory_update_files = list(results_dir.glob("experiment_trajectory_update_*.csv"))

if not step_pid_files or not oneshot_files or not trajectory_update_files:
    print("Error: Could not find all three experiment files")
    exit(1)

# Get latest files
latest_step_pid = max(step_pid_files, key=lambda p: p.stat().st_mtime)
latest_oneshot = max(oneshot_files, key=lambda p: p.stat().st_mtime)
latest_trajectory_update = max(trajectory_update_files, key=lambda p: p.stat().st_mtime)

print("="*80)
print("STEADY-STATE ERROR DURING DISTURBANCE ANALYSIS")
print("="*80)
print(f"\nFiles analyzed:")
print(f"  Step PID: {latest_step_pid}")
print(f"  One-Shot Trajectory: {latest_oneshot}")
print(f"  Trajectory Update: {latest_trajectory_update}")

# Disturbance period (sinusoidal from t=5.0s to t=8.0s)
disturbance_start = 5.0
disturbance_end = 8.0

def analyze_steady_state_error(filename, method_name):
    """Analyze steady-state error during disturbance period."""
    time_data = []
    position_data = []
    setpoint_data = []
    error_data = []
    
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['time'])
            if disturbance_start <= t <= disturbance_end:
                time_data.append(t)
                position_data.append(float(row['position_x']))
                setpoint_data.append(float(row['setpoint']))
                error_data.append(float(row['error']))
    
    if len(time_data) == 0:
        return None
    
    time_data = np.array(time_data)
    position_data = np.array(position_data) * 100  # Convert to cm
    setpoint_data = np.array(setpoint_data) * 100  # Convert to cm
    error_data = np.array(error_data) * 100  # Convert to cm
    
    # Calculate statistics
    mean_error = np.mean(error_data)
    rms_error = np.sqrt(np.mean(error_data**2))
    max_error = np.max(np.abs(error_data))
    std_error = np.std(error_data)
    
    # Steady-state error (absolute value)
    mean_abs_error = np.mean(np.abs(error_data))
    
    return {
        'method': method_name,
        'mean_error': mean_error,
        'rms_error': rms_error,
        'max_error': max_error,
        'std_error': std_error,
        'mean_abs_error': mean_abs_error,
        'data_points': len(time_data),
        'time_range': (time_data[0], time_data[-1])
    }

# Analyze all three
results = []
for filename, method_name in [
    (latest_step_pid, "Step PID"),
    (latest_oneshot, "One-Shot Trajectory"),
    (latest_trajectory_update, "Trajectory Update")
]:
    result = analyze_steady_state_error(filename, method_name)
    if result:
        results.append(result)

# Print results
print(f"\n{'='*80}")
print(f"STEADY-STATE ERROR DURING DISTURBANCE (t={disturbance_start}s to t={disturbance_end}s)")
print(f"{'='*80}")

print(f"\n{'Method':<25} {'Mean Error':<15} {'RMS Error':<15} {'Max |Error|':<15} {'Mean |Error|':<15}")
print("-"*80)
for r in results:
    print(f"{r['method']:<25} {r['mean_error']:>8.3f} cm    {r['rms_error']:>8.3f} cm    "
          f"{r['max_error']:>8.3f} cm    {r['mean_abs_error']:>8.3f} cm")

print(f"\n{'='*80}")
print("DETAILED STATISTICS:")
print(f"{'='*80}")
for r in results:
    print(f"\n{r['method']}:")
    print(f"  Data points: {r['data_points']}")
    print(f"  Time range: {r['time_range'][0]:.3f}s to {r['time_range'][1]:.3f}s")
    print(f"  Mean error: {r['mean_error']:.4f} cm")
    print(f"  RMS error: {r['rms_error']:.4f} cm")
    print(f"  Max |error|: {r['max_error']:.4f} cm")
    print(f"  Mean |error|: {r['mean_abs_error']:.4f} cm")
    print(f"  Std dev: {r['std_error']:.4f} cm")

print(f"\n{'='*80}")
print("INTERPRETATION:")
print(f"{'='*80}")
print("Steady-state error during disturbance indicates how well each method")
print("maintains the target position (0cm) while the platform is being")
print("continuously disturbed (sinusoidal oscillation).")
print("\nLower values = better disturbance rejection")
print("Mean |error| is the most relevant metric (average deviation from target)")

