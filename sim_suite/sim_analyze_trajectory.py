# Quick analysis of trajectory update data (works with both real and simulated experiments)
import csv
import numpy as np
import sys
import os

# Allow command-line argument for filename
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    # Default to latest trajectory update experiment
    results_dir = "results"
    if not os.path.exists(results_dir):
        results_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "results")
    
    # Find latest trajectory update file
    import glob
    files = glob.glob(os.path.join(results_dir, "experiment_trajectory_update_*.csv"))
    if files:
        filename = max(files, key=os.path.getmtime)
        print(f"[INFO] Using latest file: {os.path.basename(filename)}")
    else:
        print("[ERROR] No trajectory update experiment files found.")
        print("Usage: python sim_analyze_trajectory.py <filename.csv>")
        sys.exit(1)

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
print("TRAJECTORY UPDATE DATA ANALYSIS")
print("="*80)
print(f"\nFile: {filename}")
print(f"Total samples: {len(time_data)}")
print(f"Time range: {time_data[0]:.2f}s to {time_data[-1]:.2f}s")
print(f"Duration: {time_data[-1] - time_data[0]:.2f}s")
print(f"\nPosition range: {position_data.min()*100:.2f}cm to {position_data.max()*100:.2f}cm")
print(f"Setpoint range: {setpoint_data.min()*100:.2f}cm to {setpoint_data.max()*100:.2f}cm")

print("\n" + "="*80)
print("SETPOINT ANALYSIS")
print("="*80)

# Check if setpoints are making progress toward 0
print(f"\nInitial position: {position_data[0]*100:.2f}cm")
print(f"Final position: {position_data[-1]*100:.2f}cm")
print(f"Initial setpoint: {setpoint_data[0]*100:.2f}cm")
print(f"Final setpoint: {setpoint_data[-1]*100:.2f}cm")

# Count how many times setpoint is exactly 0
zero_setpoints = np.sum(setpoint_data == 0.0)
print(f"\nSetpoint = 0.0 count: {zero_setpoints} / {len(setpoint_data)} ({100*zero_setpoints/len(setpoint_data):.1f}%)")

# Check setpoint jumps (regeneration events)
setpoint_changes = np.diff(setpoint_data)
large_jumps = np.abs(setpoint_changes) > 0.01  # > 1cm change
print(f"Large setpoint jumps (>1cm): {np.sum(large_jumps)}")

# Sample some setpoint values
print("\nFirst 20 setpoints (cm):")
for i in range(min(20, len(setpoint_data))):
    print(f"  t={time_data[i]:.3f}s: setpoint={setpoint_data[i]*100:.2f}cm, position={position_data[i]*100:.2f}cm")

print("\nLast 20 setpoints (cm):")
for i in range(max(0, len(setpoint_data)-20), len(setpoint_data)):
    print(f"  t={time_data[i]:.3f}s: setpoint={setpoint_data[i]*100:.2f}cm, position={position_data[i]*100:.2f}cm")

# Check if setpoint is ahead of position (should be for lookahead)
print("\n" + "="*80)
print("LOOKAHEAD CHECK")
print("="*80)
setpoint_ahead = setpoint_data < position_data  # Setpoint should be closer to 0 (smaller) than position
print(f"Setpoint ahead of position: {np.sum(setpoint_ahead)} / {len(setpoint_data)} ({100*np.sum(setpoint_ahead)/len(setpoint_data):.1f}%)")
print(f"Setpoint behind position: {np.sum(~setpoint_ahead)} / {len(setpoint_data)} ({100*np.sum(~setpoint_ahead)/len(setpoint_data):.1f}%)")

# Check when position is near 0
near_zero = np.abs(position_data) < 0.01  # Within 1cm of center
print(f"\nPosition near center (<1cm): {np.sum(near_zero)} samples")
if np.any(near_zero):
    first_near_zero_idx = np.where(near_zero)[0][0]
    print(f"  First time near center: t={time_data[first_near_zero_idx]:.2f}s")
    print(f"  Setpoint at that time: {setpoint_data[first_near_zero_idx]*100:.2f}cm")
