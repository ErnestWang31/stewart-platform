# Check if ball is oscillating, causing upward setpoint jumps
import csv
import numpy as np
from pathlib import Path

# Find latest trajectory update CSV
results_dir = Path("results")
csv_files = list(results_dir.glob("experiment_trajectory_update_*.csv"))
latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)

# Load data
time_data = []
position_data = []
setpoint_data = []

with open(latest_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_data.append(float(row['time']))
        position_data.append(float(row['position_x']))
        setpoint_data.append(float(row['setpoint']))

time_data = np.array(time_data)
position_data = np.array(position_data) * 100
setpoint_data = np.array(setpoint_data) * 100

print("="*80)
print("BALL OSCILLATION ANALYSIS")
print("="*80)

# Check for position reversals (ball moving backward)
position_velocity = np.diff(position_data) / np.diff(time_data)
reversals = np.where(position_velocity > 0)[0]  # Positive velocity = moving away from 0

print(f"\nPosition reversals (moving away from target): {len(reversals)}")
print(f"Percentage of time moving backward: {len(reversals)/len(position_velocity)*100:.1f}%")

if len(reversals) > 0:
    print(f"\nFirst 10 reversals:")
    for i in reversals[:10]:
        print(f"  t={time_data[i]:.3f}s: position={position_data[i]:.2f}cm, velocity={position_velocity[i]:.2f}cm/s")

# Check setpoint jumps at reversal times
print(f"\nSetpoint behavior during reversals:")
for i in reversals[:5]:
    if i < len(setpoint_data) - 1:
        setpoint_change = setpoint_data[i+1] - setpoint_data[i]
        print(f"  t={time_data[i]:.3f}s: position={position_data[i]:.2f}cm, "
              f"setpoint change={setpoint_change:+.2f}cm")

# Check correlation: when ball moves backward, does setpoint jump up?
print(f"\nCorrelation analysis:")
upward_setpoint_jumps = np.where(np.diff(setpoint_data) > 0.3)[0]  # >0.3cm upward jump
print(f"Upward setpoint jumps: {len(upward_setpoint_jumps)}")
print(f"Position reversals: {len(reversals)}")

# Check if upward jumps happen near reversals
matches = 0
for jump_idx in upward_setpoint_jumps[:10]:
    # Check if there's a reversal within 0.1s before the jump
    nearby_reversals = reversals[(time_data[reversals] >= time_data[jump_idx] - 0.1) & 
                                 (time_data[reversals] <= time_data[jump_idx] + 0.1)]
    if len(nearby_reversals) > 0:
        matches += 1
        print(f"  t={time_data[jump_idx]:.3f}s: Setpoint jump up, nearby reversal at t={time_data[nearby_reversals[0]]:.3f}s")

print(f"\nMatches: {matches}/{min(10, len(upward_setpoint_jumps))}")

