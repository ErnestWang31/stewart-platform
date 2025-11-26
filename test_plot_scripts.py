# Test that plotting scripts read data correctly
import csv
import numpy as np
from pathlib import Path

# Test all three types
experiment_types = [
    ("experiment_step_pid_*.csv", "Step PID"),
    ("experiment_oneshot_trajectory_*.csv", "One-Shot Trajectory"),
    ("experiment_trajectory_update_*.csv", "Trajectory Update")
]

results_dir = Path("results")

for pattern, name in experiment_types:
    csv_files = list(results_dir.glob(pattern))
    if not csv_files:
        print(f"{name}: No files found")
        continue
    
    latest_file = max(csv_files, key=lambda p: p.stat().st_mtime)
    print(f"\n{name}:")
    print(f"  File: {latest_file}")
    
    # Read first few rows
    with open(latest_file, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        
        if len(rows) == 0:
            print("  ERROR: No data rows!")
            continue
        
        # Check columns
        first_row = rows[0]
        print(f"  Columns: {list(first_row.keys())}")
        
        # Check data types and values
        print(f"  First row:")
        print(f"    time: {first_row['time']} (type: {type(first_row['time'])})")
        print(f"    position_x: {first_row['position_x']} (type: {type(first_row['position_x'])})")
        print(f"    setpoint: {first_row['setpoint']} (type: {type(first_row['setpoint'])})")
        
        # Check if setpoint column exists
        if 'setpoint' not in first_row:
            print("  ERROR: 'setpoint' column not found!")
            print(f"  Available columns: {list(first_row.keys())}")
        else:
            # Try to read all data
            time_data = []
            position_data = []
            setpoint_data = []
            
            for row in rows:
                try:
                    time_data.append(float(row['time']))
                    position_data.append(float(row['position_x']))
                    setpoint_data.append(float(row['setpoint']))
                except (KeyError, ValueError) as e:
                    print(f"  ERROR reading row: {e}")
                    break
            
            if len(time_data) > 0:
                print(f"  Successfully loaded {len(time_data)} rows")
                print(f"    Time range: {min(time_data):.3f}s to {max(time_data):.3f}s")
                print(f"    Position range: {min(position_data)*100:.2f}cm to {max(position_data)*100:.2f}cm")
                print(f"    Setpoint range: {min(setpoint_data)*100:.2f}cm to {max(setpoint_data)*100:.2f}cm")
                
                # Check for any issues
                if len(setpoint_data) != len(time_data):
                    print(f"  WARNING: Mismatch in data lengths!")
                if any(np.isnan(s) for s in setpoint_data):
                    print(f"  WARNING: NaN values in setpoint data!")
                if any(np.isinf(s) for s in setpoint_data):
                    print(f"  WARNING: Inf values in setpoint data!")

print("\n" + "="*80)
print("All plotting scripts should read:")
print("  - Column: 'setpoint'")
print("  - Data type: float (in meters)")
print("  - Conversion: multiply by 100 to get cm")
print("="*80)

