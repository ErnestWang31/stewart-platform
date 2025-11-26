# Analyze why ball stays at one spot at start
import csv
import numpy as np

filename = "results/experiment_oneshot_trajectory_20251125_200833.csv"

time_data = []
position_data = []
setpoint_data = []
error_data = []
control_data = []

with open(filename, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_data.append(float(row['time']))
        position_data.append(float(row['position_x']))
        setpoint_data.append(float(row['setpoint']))
        error_data.append(float(row['error']))
        control_data.append(float(row['control_signal']))

time_data = np.array(time_data)
position_data = np.array(position_data)
setpoint_data = np.array(setpoint_data)
error_data = np.array(error_data)
control_data = np.array(control_data)

print("="*80)
print("STALL ANALYSIS - Why ball stays at one spot")
print("="*80)

# Find when ball actually starts moving
position_change = np.abs(np.diff(position_data))
movement_threshold = 0.001  # 1mm movement
significant_movement = np.where(position_change > movement_threshold)[0]

if len(significant_movement) > 0:
    first_movement_idx = significant_movement[0]
    first_movement_time = time_data[first_movement_idx]
    print(f"\nFirst significant movement (>1mm): t={first_movement_time:.3f}s")
    print(f"  Position before: {position_data[first_movement_idx]*100:.2f}cm")
    print(f"  Position after: {position_data[first_movement_idx+1]*100:.2f}cm")
    print(f"  Change: {position_change[first_movement_idx]*100:.2f}cm")
else:
    print("\nNo significant movement detected!")

# Analyze control signal during stall period
stall_period = 1.5  # First 1.5 seconds
stall_mask = time_data < stall_period
stall_control = control_data[stall_mask]
stall_error = error_data[stall_mask]
stall_position = position_data[stall_mask]

print(f"\n{'='*80}")
print(f"STALL PERIOD ANALYSIS (first {stall_period}s)")
print(f"{'='*80}")
print(f"Average position: {np.mean(stall_position)*100:.2f}cm")
print(f"Position std dev: {np.std(stall_position)*100:.3f}cm")
print(f"Position range: {(np.max(stall_position) - np.min(stall_position))*100:.2f}cm")
print(f"\nAverage error: {np.mean(stall_error)*100:.2f}cm")
print(f"Max error: {np.max(np.abs(stall_error))*100:.2f}cm")
print(f"\nAverage control signal: {np.mean(stall_control):.2f}°")
print(f"Max control signal: {np.max(np.abs(stall_control)):.2f}°")
print(f"Min control signal: {np.min(stall_control):.2f}°")
print(f"Control signal std dev: {np.std(stall_control):.2f}°")

# Check if control signal is increasing
print(f"\nControl signal trend:")
print(f"  First 0.5s avg: {np.mean(control_data[time_data < 0.5]):.2f}°")
print(f"  0.5-1.0s avg: {np.mean(control_data[(time_data >= 0.5) & (time_data < 1.0)]):.2f}°")
print(f"  1.0-1.5s avg: {np.mean(control_data[(time_data >= 1.0) & (time_data < 1.5)]):.2f}°")

# Check error vs control relationship
print(f"\nError vs Control relationship:")
mask1 = np.abs(error_data + 0.01) < 0.002
mask2 = np.abs(error_data + 0.02) < 0.002
mask3 = np.abs(error_data + 0.03) < 0.002
if np.any(mask1):
    print(f"  When error = -1cm, control ~ {np.mean(control_data[mask1]):.2f}°")
if np.any(mask2):
    print(f"  When error = -2cm, control ~ {np.mean(control_data[mask2]):.2f}°")
if np.any(mask3):
    print(f"  When error = -3cm, control ~ {np.mean(control_data[mask3]):.2f}°")

# Find when control signal becomes large enough (e.g., >3°)
large_control_idx = np.where(np.abs(control_data) > 3.0)[0]
if len(large_control_idx) > 0:
    first_large_control_time = time_data[large_control_idx[0]]
    print(f"\nFirst time control signal >3°: t={first_large_control_time:.3f}s")
    print(f"  Control: {control_data[large_control_idx[0]]:.2f}°")
    print(f"  Error: {error_data[large_control_idx[0]]*100:.2f}cm")
    print(f"  Position: {position_data[large_control_idx[0]]*100:.2f}cm")

# Check for deadband or minimum control threshold
print(f"\n{'='*80}")
print("MECHANICAL ANALYSIS")
print(f"{'='*80}")
print("Possible causes:")
print("  1. Static friction / stiction - control signal too small to overcome")
print("  2. PID gains too low - not generating enough control effort")
print("  3. Deadband in actuators - small signals don't move platform")
print("  4. Ball stuck or high friction surface")

# Calculate effective gain (control/error)
effective_gains = control_data / (error_data + 1e-6)  # Avoid division by zero
print(f"\nEffective gain (control/error) during stall:")
print(f"  Mean: {np.mean(effective_gains[stall_mask]):.2f}°/cm")
print(f"  This should match Kp if PID is working correctly")

