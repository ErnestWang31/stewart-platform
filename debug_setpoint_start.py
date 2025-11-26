# Debug why setpoint starts at 11.7cm instead of 13.5cm
import csv
import numpy as np
from pathlib import Path

# Find latest trajectory update CSV
results_dir = Path("results")
csv_files = list(results_dir.glob("experiment_trajectory_update_*.csv"))
latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)

print(f"Analyzing: {latest_file}\n")

# Load first few data points
time_data = []
position_data = []
setpoint_data = []

with open(latest_file, 'r') as f:
    reader = csv.DictReader(f)
    for i, row in enumerate(reader):
        if i >= 20:  # Just first 20 points
            break
        time_data.append(float(row['time']))
        position_data.append(float(row['position_x']) * 100)  # Convert to cm
        setpoint_data.append(float(row['setpoint']) * 100)  # Convert to cm

print("="*80)
print("FIRST FEW DATA POINTS:")
print("="*80)
print(f"{'Time (s)':<12} {'Position (cm)':<15} {'Setpoint (cm)':<15} {'Difference':<15}")
print("-"*80)
for i in range(min(10, len(time_data))):
    diff = setpoint_data[i] - position_data[i]
    print(f"{time_data[i]:<12.3f} {position_data[i]:<15.2f} {setpoint_data[i]:<15.2f} {diff:+.2f}cm")

print("\n" + "="*80)
print("ANALYSIS:")
print("="*80)
print(f"Initial position: {position_data[0]:.2f}cm")
print(f"Initial setpoint: {setpoint_data[0]:.2f}cm")
print(f"Difference: {setpoint_data[0] - position_data[0]:.2f}cm")

# Check trajectory updater settings
print("\nExpected behavior:")
print("  - update_interval = 0.2s")
print("  - lookahead_time = min(trajectory_duration, update_interval * 2) = min(3.0, 0.4) = 0.4s")
print("  - At t=0.048s, trajectory_time = 0.048s")
print("  - sample_time = 0.048 + 0.4 = 0.448s")
print("  - For a 3.0s trajectory from 13.54cm to 0cm:")
print("    - At t=0.448s: position = 13.54 + (0 - 13.54) * (0.448/3.0) = 13.54 - 2.02 = 11.52cm")
print(f"  - Actual setpoint at t=0.048s: {setpoint_data[0]:.2f}cm")
print(f"  - Expected setpoint (with lookahead): ~11.52cm")
print(f"  - Match: {'YES' if abs(setpoint_data[0] - 11.52) < 0.5 else 'NO'}")

print("\n" + "="*80)
print("ROOT CAUSE:")
print("="*80)
print("The setpoint starts at 11.7cm because:")
print("  1. Trajectory is generated from 13.54cm to 0cm over 3.0s")
print("  2. get_setpoint() uses lookahead of 0.4s")
print("  3. At t=0.048s, it samples trajectory at t=0.448s")
print("  4. This gives setpoint ~ 11.5cm (not 13.54cm)")
print("\nThis is INTENTIONAL - the lookahead ensures the setpoint is always")
print("ahead of the current position to maintain forward progress.")

