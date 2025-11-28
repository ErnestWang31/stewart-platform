# Plot Position vs Setpoint for Trajectory Update Experiment
# Finds and plots the latest Trajectory Update experiment with trajectory segments

import csv
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

try:
    from trajectory_generator import generate_trajectory
except ImportError:
    print("Warning: trajectory_generator not found, will skip trajectory segments")
    generate_trajectory = None

# Check if file path provided as argument
if len(sys.argv) > 1:
    latest_file = Path(sys.argv[1])
    if not latest_file.exists():
        print(f"Error: File not found: {latest_file}")
        exit(1)
    print(f"Loading: {latest_file}")
    # Try to find segments file in same directory
    timestamp = latest_file.stem.replace("experiment_trajectory_update_", "")
    segments_file = latest_file.parent / f"experiment_trajectory_update_{timestamp}_segments.json"
    if not segments_file.exists():
        # Try in results directory
        results_dir = Path("results")
        segments_file = results_dir / f"experiment_trajectory_update_{timestamp}_segments.json"
else:
    # Find latest Trajectory Update CSV file
    results_dir = Path("results")
    csv_files = list(results_dir.glob("experiment_trajectory_update_*.csv"))
    
    if not csv_files:
        print("No Trajectory Update experiment files found in results/")
        exit(1)
    
    # Get latest file (by modification time)
    latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)
    print(f"Loading: {latest_file}")
    
    # Find corresponding segments JSON file
    timestamp = latest_file.stem.replace("experiment_trajectory_update_", "")
    segments_file = results_dir / f"experiment_trajectory_update_{timestamp}_segments.json"

# Load CSV data
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

# Load trajectory segments
segments = []
if segments_file.exists():
    with open(segments_file, 'r') as f:
        segments = json.load(f)
    print(f"Loaded {len(segments)} trajectory segments from {segments_file}")
else:
    print(f"Warning: Segments file not found: {segments_file}")

# Plot
plt.figure(figsize=(12, 7))

# Plot trajectory segments
if segments and generate_trajectory:
    for i, segment in enumerate(segments):
        x0 = segment['x0'] * 100  # Convert to cm
        xf = segment['xf'] * 100
        start_time = segment['start_time']
        duration = segment['duration']
        method = segment.get('method', 'linear')
        curvature = segment.get('curvature', 3.0)
        
        # Generate trajectory for this segment
        traj = generate_trajectory(x0/100, xf/100, duration, method=method, curvature=curvature)
        
        # Sample trajectory at multiple points
        segment_times = np.linspace(0, duration, max(50, int(duration * 20)))
        segment_positions = [traj.get_position(t) * 100 for t in segment_times]
        segment_absolute_times = start_time + segment_times
        
        # Plot segment (use different colors/linestyles for visibility)
        color = plt.cm.viridis(i / max(1, len(segments) - 1))
        plt.plot(segment_absolute_times, segment_positions, 
                color=color, linestyle=':', linewidth=1.5, alpha=0.6,
                label=f'Segment {i+1}' if i < 5 else None)  # Only label first 5 to avoid clutter

# Plot actual position and setpoint
plt.plot(time_data, position_data, 'b-', label='Actual Position', linewidth=2.5)
plt.plot(time_data, setpoint_data, 'r--', label='Setpoint', linewidth=2)

plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Position (cm)', fontsize=12)
plt.title('Trajectory Update Experiment - Position vs Setpoint with Trajectory Segments', 
          fontsize=14, fontweight='bold')
plt.legend(fontsize=10, loc='best')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

print("Plot displayed")

