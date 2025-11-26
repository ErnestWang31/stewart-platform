# Analyze oneshot trajectory experiment start behavior
import csv
import numpy as np

filename = "results/experiment_oneshot_trajectory_20251125_195930.csv"

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
print("ONE-SHOT TRAJECTORY START ANALYSIS")
print("="*80)

print(f"\nTotal samples: {len(time_data)}")
print(f"Time range: {time_data[0]:.3f}s to {time_data[-1]:.2f}s")

print("\n" + "="*80)
print("FIRST 30 DATA POINTS (Start of Experiment)")
print("="*80)
print(f"{'Time (s)':<10} {'Position (cm)':<15} {'Setpoint (cm)':<15} {'Error (cm)':<15} {'Control (°)':<15}")
print("-" * 80)

for i in range(min(30, len(time_data))):
    print(f"{time_data[i]:<10.3f} {position_data[i]*100:<15.2f} {setpoint_data[i]*100:<15.2f} "
          f"{error_data[i]*100:<15.2f} {control_data[i]:<15.2f}")

print("\n" + "="*80)
print("ANALYSIS")
print("="*80)

# Check initial conditions
print(f"\nInitial position: {position_data[0]*100:.2f} cm")
print(f"Initial setpoint: {setpoint_data[0]*100:.2f} cm")
print(f"Initial error: {error_data[0]*100:.2f} cm")
print(f"Initial control: {control_data[0]:.2f}°")

# Check trajectory start
print(f"\nFirst setpoint change:")
for i in range(min(10, len(setpoint_data))):
    if i > 0 and abs(setpoint_data[i] - setpoint_data[i-1]) > 0.001:
        print(f"  At t={time_data[i]:.3f}s: setpoint changed from {setpoint_data[i-1]*100:.2f}cm to {setpoint_data[i]*100:.2f}cm")
        print(f"    Position: {position_data[i]*100:.2f}cm")
        print(f"    Error: {error_data[i]*100:.2f}cm")
        print(f"    Control: {control_data[i]:.2f}°")
        break

# Check if setpoint is following trajectory
print(f"\nSetpoint trajectory check (first 2 seconds):")
mask = time_data <= 2.0
if np.any(mask):
    times = time_data[mask]
    setpoints = setpoint_data[mask]
    positions = position_data[mask]
    
    # Expected trajectory: linear from initial to 0 over trajectory_duration
    # Assuming trajectory_duration = 3.0s (from code)
    initial_pos = position_data[0]
    trajectory_duration = 3.0
    
    print(f"  Initial position: {initial_pos*100:.2f}cm")
    print(f"  Expected trajectory: r(t) = {initial_pos*100:.2f} + (0 - {initial_pos*100:.2f}) * (t / {trajectory_duration})")
    
    print(f"\n  First 10 setpoints vs expected:")
    for i in range(min(10, len(times))):
        expected = initial_pos + (0.0 - initial_pos) * (times[i] / trajectory_duration) if trajectory_duration > 0 else 0.0
        print(f"    t={times[i]:.3f}s: setpoint={setpoints[i]*100:.2f}cm, expected={expected*100:.2f}cm, diff={abs(setpoints[i]-expected)*100:.3f}cm")

# Check control response
print(f"\nControl signal analysis (first 1 second):")
mask = time_data <= 1.0
if np.any(mask):
    times_1s = time_data[mask]
    controls_1s = control_data[mask]
    errors_1s = error_data[mask]
    
    print(f"  Max control: {np.max(np.abs(controls_1s)):.2f}°")
    print(f"  Max error: {np.max(np.abs(errors_1s))*100:.2f}cm")
    print(f"  Control at t=0: {controls_1s[0]:.2f}°")
    print(f"  Error at t=0: {errors_1s[0]*100:.2f}cm")

