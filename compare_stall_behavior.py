# Compare stall behavior across all three methods
import csv
import numpy as np
from pathlib import Path

# Latest files
step_pid_file = "results/experiment_step_pid_20251125_193524.csv"
oneshot_file = "results/experiment_oneshot_trajectory_20251125_200833.csv"
trajectory_update_file = "results/experiment_trajectory_update_20251125_200810.csv"

def analyze_stall(filename, method_name):
    """Analyze stall behavior in a CSV file."""
    time_data = []
    position_data = []
    control_data = []
    
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_data.append(float(row['time']))
            position_data.append(float(row['position_x']))
            control_data.append(float(row['control_signal']))
    
    time_data = np.array(time_data)
    position_data = np.array(position_data)
    control_data = np.array(control_data)
    
    # Find when ball actually starts moving
    position_change = np.abs(np.diff(position_data))
    movement_threshold = 0.001  # 1mm movement
    significant_movement = np.where(position_change > movement_threshold)[0]
    
    if len(significant_movement) > 0:
        first_movement_idx = significant_movement[0]
        first_movement_time = time_data[first_movement_idx]
    else:
        first_movement_time = None
    
    # Analyze first 1.5 seconds
    stall_period = 1.5
    stall_mask = time_data < stall_period
    stall_position = position_data[stall_mask]
    stall_control = control_data[stall_mask]
    
    position_std = np.std(stall_position) if len(stall_position) > 0 else 0
    position_range = np.max(stall_position) - np.min(stall_position) if len(stall_position) > 0 else 0
    max_control = np.max(np.abs(stall_control)) if len(stall_control) > 0 else 0
    
    return {
        'method': method_name,
        'first_movement_time': first_movement_time,
        'position_std': position_std,
        'position_range': position_range,
        'max_control_during_stall': max_control,
        'stalled': first_movement_time is None or first_movement_time > 0.5
    }

print("="*80)
print("STALL BEHAVIOR COMPARISON")
print("="*80)

results = []
for filename, method_name in [
    (step_pid_file, "Step PID"),
    (oneshot_file, "One-Shot Trajectory"),
    (trajectory_update_file, "Trajectory Update")
]:
    try:
        result = analyze_stall(filename, method_name)
        results.append(result)
        print(f"\n{method_name}:")
        print(f"  First movement: {result['first_movement_time']:.3f}s" if result['first_movement_time'] else "  First movement: N/A")
        print(f"  Position std dev (first 1.5s): {result['position_std']*100:.3f}cm")
        print(f"  Position range (first 1.5s): {result['position_range']*100:.2f}cm")
        print(f"  Max control during stall: {result['max_control']:.2f}°")
        print(f"  Stalled (>0.5s delay): {'YES' if result['stalled'] else 'NO'}")
    except FileNotFoundError:
        print(f"\n{method_name}: File not found")
    except Exception as e:
        print(f"\n{method_name}: Error - {e}")

print("\n" + "="*80)
print("RECOMMENDATION:")
print("="*80)

if all(r['stalled'] for r in results if 'stalled' in r):
    print("All methods show stall behavior - this is fair for comparison")
    print("  Static friction affects all methods equally")
    print("  You can compare them as-is, or fix it for all methods")
elif any(r['stalled'] for r in results if 'stalled' in r):
    print("WARNING: Some methods stall, others don't - comparison may be unfair")
    print("  Consider fixing static friction for all methods")
    print("  OR note this difference in your comparison report")
else:
    print("No significant stall behavior detected")

