# Analyze timing issue in oneshot trajectory
import csv
import numpy as np

filename = "results/experiment_oneshot_trajectory_20251125_195930.csv"

time_data = []
position_data = []
setpoint_data = []

with open(filename, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_data.append(float(row['time']))
        position_data.append(float(row['position_x']))
        setpoint_data.append(float(row['setpoint']))

time_data = np.array(time_data)
position_data = np.array(position_data)
setpoint_data = np.array(setpoint_data)

print("="*80)
print("TIMING ANALYSIS")
print("="*80)

# The trajectory should be: r(t) = x0 + (xf - x0) * (t/T)
# We know: xf = 0, T = 3.0s (from code)
# We need to find x0

# Try to reverse-engineer x0 from first setpoint
# r(0.048) = x0 + (0 - x0) * (0.048 / 3.0) = x0 * (1 - 0.048/3.0) = x0 * 0.984
# So: x0 = r(0.048) / 0.984

first_setpoint = setpoint_data[0]
first_time = time_data[0]
trajectory_duration = 3.0

# Reverse calculate initial position
# r(t) = x0 + (0 - x0) * (t/T) = x0 * (1 - t/T)
# So: x0 = r(t) / (1 - t/T)
calculated_x0 = first_setpoint / (1 - first_time / trajectory_duration)

print(f"\nFirst data point:")
print(f"  Time: {first_time:.3f}s")
print(f"  Setpoint: {first_setpoint*100:.2f}cm")
print(f"  Position: {position_data[0]*100:.2f}cm")

print(f"\nReverse-calculated initial position from setpoint:")
print(f"  x0 = {first_setpoint*100:.2f} / (1 - {first_time:.3f}/{trajectory_duration})")
print(f"  x0 = {calculated_x0*100:.2f}cm")

print(f"\nActual initial position from data: {position_data[0]*100:.2f}cm")
print(f"Difference: {abs(calculated_x0 - position_data[0])*100:.2f}cm")

# Check if trajectory matches expected
print(f"\nChecking trajectory consistency:")
print(f"  If x0 = {calculated_x0*100:.2f}cm, T = {trajectory_duration}s:")
for i in range(min(5, len(time_data))):
    expected = calculated_x0 * (1 - time_data[i] / trajectory_duration)
    actual = setpoint_data[i]
    diff = abs(expected - actual) * 100
    print(f"    t={time_data[i]:.3f}s: expected={expected*100:.2f}cm, actual={actual*100:.2f}cm, diff={diff:.3f}cm")

# Check if there's a time offset
print(f"\nChecking for time offset:")
# If trajectory was generated at t=-0.048s, then at t=0.048s we'd be at t=0.096s in trajectory time
# r(0.096) = x0 * (1 - 0.096/3.0) = x0 * 0.968
# If x0 = 13.82cm: r(0.096) = 13.38cm
# But we see 13.47cm, which is between 13.60cm (t=0.048) and 13.38cm (t=0.096)

# Try: what if trajectory started before t=0?
print(f"  If trajectory started at t=-0.048s (before experiment start):")
print(f"    At t=0.048s (experiment time), trajectory time = 0.096s")
x0_guess = 13.82 / 100  # meters
r_at_096 = x0_guess * (1 - 0.096 / trajectory_duration)
print(f"    Expected setpoint: {r_at_096*100:.2f}cm")
print(f"    Actual setpoint: {setpoint_data[0]*100:.2f}cm")

print(f"\n  If trajectory started at t=0 (at experiment start):")
r_at_048 = x0_guess * (1 - 0.048 / trajectory_duration)
print(f"    Expected setpoint: {r_at_048*100:.2f}cm")
print(f"    Actual setpoint: {setpoint_data[0]*100:.2f}cm")

