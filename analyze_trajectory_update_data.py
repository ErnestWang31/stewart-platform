# Analyze trajectory update experiment data to verify behavior
import csv
import numpy as np
from pathlib import Path

# Find latest trajectory update CSV
results_dir = Path("results")
csv_files = list(results_dir.glob("experiment_trajectory_update_*.csv"))
if not csv_files:
    print("No trajectory update files found")
    exit(1)

latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)
print(f"Analyzing: {latest_file}\n")

# Load data
time_data = []
position_data = []
setpoint_data = []
error_data = []

with open(latest_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_data.append(float(row['time']))
        position_data.append(float(row['position_x']))
        setpoint_data.append(float(row['setpoint']))
        error_data.append(float(row['error']))

time_data = np.array(time_data)
position_data = np.array(position_data) * 100  # Convert to cm
setpoint_data = np.array(setpoint_data) * 100  # Convert to cm
error_data = np.array(error_data) * 100  # Convert to cm

print("="*80)
print("TRAJECTORY UPDATE BEHAVIOR ANALYSIS")
print("="*80)

# 1. Check trajectory regeneration frequency
# Look for significant setpoint changes (indicates new trajectory)
setpoint_changes = np.abs(np.diff(setpoint_data))
significant_changes = np.where(setpoint_changes > 0.5)[0]  # >0.5cm change

print(f"\n1. TRAJECTORY REGENERATION:")
print(f"   Total data points: {len(time_data)}")
print(f"   Significant setpoint changes (>0.5cm): {len(significant_changes)}")
if len(significant_changes) > 0:
    change_times = time_data[significant_changes]
    intervals = np.diff(change_times)
    print(f"   Average interval between changes: {np.mean(intervals):.3f}s")
    print(f"   Expected update interval: 0.2s")
    print(f"   First few change times: {change_times[:5]}")
else:
    print("   No significant setpoint changes detected")

# 2. Check if setpoints are converging to target (0cm)
print(f"\n2. SETPOINT CONVERGENCE TO TARGET:")
print(f"   Initial setpoint: {setpoint_data[0]:.2f}cm")
print(f"   Final setpoint: {setpoint_data[-1]:.2f}cm")
print(f"   Target: 0.0cm")
print(f"   Setpoint at t=1.0s: {setpoint_data[time_data < 1.0][-1] if np.any(time_data < 1.0) else 'N/A'}")
print(f"   Setpoint at t=2.0s: {setpoint_data[time_data < 2.0][-1] if np.any(time_data < 2.0) else 'N/A'}")
print(f"   Setpoint at t=3.0s: {setpoint_data[time_data < 3.0][-1] if np.any(time_data < 3.0) else 'N/A'}")

# Check if setpoints are decreasing (moving toward 0)
setpoint_trend = np.polyfit(time_data[:min(100, len(time_data))], setpoint_data[:min(100, len(time_data))], 1)
print(f"   Setpoint trend (slope): {setpoint_trend[0]:.3f} cm/s (negative = converging)")

# 3. Check lookahead behavior
# Lookahead should make setpoint always ahead of position
print(f"\n3. LOOKAHEAD BEHAVIOR:")
position_ahead = setpoint_data - position_data  # Should be positive if setpoint is ahead
positive_lookahead = np.sum(position_ahead > 0) / len(position_ahead) * 100
print(f"   Setpoint ahead of position: {positive_lookahead:.1f}% of time")
print(f"   Average lookahead distance: {np.mean(position_ahead[position_ahead > 0]):.2f}cm" if np.any(position_ahead > 0) else "   No positive lookahead")
print(f"   Max lookahead: {np.max(position_ahead):.2f}cm")
print(f"   Min lookahead: {np.min(position_ahead):.2f}cm")

# 4. Check remaining time logic
# If use_remaining_time=True, setpoints should converge to 0 at t=3.0s
print(f"\n4. REMAINING TIME LOGIC:")
target_time = 3.0  # total_trajectory_duration
if np.any(time_data >= target_time):
    setpoint_at_target_time = setpoint_data[time_data >= target_time][0]
    print(f"   Setpoint at t={target_time}s: {setpoint_at_target_time:.2f}cm")
    print(f"   Expected: 0.0cm")
    print(f"   Difference: {abs(setpoint_at_target_time):.2f}cm")
else:
    print(f"   Experiment didn't reach t={target_time}s")

# 5. Check trajectory segment behavior
# Look for setpoint jumps (new trajectory starting)
print(f"\n5. TRAJECTORY SEGMENT ANALYSIS:")
# Find times when setpoint jumps significantly (new trajectory)
for i in range(1, min(20, len(setpoint_data))):  # Check first 20 points
    if abs(setpoint_data[i] - setpoint_data[i-1]) > 0.3:  # Significant jump
        print(f"   t={time_data[i]:.3f}s: Setpoint jump from {setpoint_data[i-1]:.2f}cm to {setpoint_data[i]:.2f}cm")
        print(f"      Position: {position_data[i]:.2f}cm, Error: {error_data[i]:.2f}cm")

# 6. Overall progress
print(f"\n6. OVERALL PROGRESS:")
print(f"   Initial position: {position_data[0]:.2f}cm")
print(f"   Final position: {position_data[-1]:.2f}cm")
print(f"   Distance traveled: {position_data[0] - position_data[-1]:.2f}cm")
print(f"   Initial error: {error_data[0]:.2f}cm")
print(f"   Final error: {error_data[-1]:.2f}cm")

# 7. Check if setpoints are smooth or jumpy
print(f"\n7. SETPOINT SMOOTHNESS:")
setpoint_velocity = np.diff(setpoint_data) / np.diff(time_data)
print(f"   Average setpoint velocity: {np.mean(setpoint_velocity):.3f} cm/s")
print(f"   Max setpoint velocity: {np.max(np.abs(setpoint_velocity)):.3f} cm/s")
print(f"   Setpoint velocity std dev: {np.std(setpoint_velocity):.3f} cm/s")
# High std dev indicates jumpy setpoints (frequent regeneration)

print("\n" + "="*80)
print("SUMMARY:")
print("="*80)
print("Expected behavior:")
print("  - Trajectory regenerated every ~0.2s")
print("  - Setpoints converge to 0cm at t=3.0s")
print("  - Setpoint should be ahead of position (lookahead)")
print("  - Smooth overall trajectory despite frequent updates")

