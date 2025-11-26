# Plot Position vs Setpoint for Step PID Experiment
# Finds and plots the latest Step PID experiment

import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Find latest Step PID CSV file
results_dir = Path("results")
csv_files = list(results_dir.glob("experiment_step_pid_*.csv"))

if not csv_files:
    print("No Step PID experiment files found in results/")
    exit(1)

# Get latest file (by modification time)
latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)
print(f"Loading: {latest_file}")

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
position_data = np.array(position_data) * 100  # Convert to cm
setpoint_data = np.array(setpoint_data) * 100  # Convert to cm

# Plot
plt.figure(figsize=(10, 6))
plt.plot(time_data, position_data, 'b-', label='Position', linewidth=2)
plt.plot(time_data, setpoint_data, 'r--', label='Setpoint', linewidth=2)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Position (cm)', fontsize=12)
plt.title('Step PID Experiment - Position vs Setpoint', fontsize=14, fontweight='bold')
plt.legend(fontsize=11)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

print("Plot displayed")

