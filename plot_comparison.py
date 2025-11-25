# Plot Comparison Module
# Generates visualization plots for controller comparison

import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from data_logger import DataLogger
from metrics import compute_smoothness

def find_latest_experiment_files():
    """Find the most recent experiment CSV files.
    
    Returns:
        tuple: (step_pid_file, oneshot_file, update_file) or None if not found
    """
    results_dir = "results"
    if not os.path.exists(results_dir):
        return None, None, None
    
    # Find files matching experiment patterns
    step_pid_files = glob.glob(os.path.join(results_dir, "experiment1_step_pid_*.csv"))
    oneshot_files = glob.glob(os.path.join(results_dir, "experiment2_trajectory_oneshot_*.csv"))
    update_files = glob.glob(os.path.join(results_dir, "experiment3_trajectory_update_*.csv"))
    
    # Get most recent files
    step_pid_file = max(step_pid_files, key=os.path.getmtime) if step_pid_files else None
    oneshot_file = max(oneshot_files, key=os.path.getmtime) if oneshot_files else None
    update_file = max(update_files, key=os.path.getmtime) if update_files else None
    
    return step_pid_file, oneshot_file, update_file


def load_experiment_data(filename):
    """Load experiment data from CSV file.
    
    Args:
        filename (str): Path to CSV file
        
    Returns:
        dict: Dictionary with data arrays
    """
    if filename is None or not os.path.exists(filename):
        return None
    
    logger = DataLogger(filename=filename)
    # Read the file to populate data
    import csv
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            logger.log(
                time=float(row['time']),
                position_x=float(row['position_x']),
                setpoint=float(row['setpoint']),
                error=float(row['error']),
                control_signal=float(row['control_signal']),
                tilt_angle=float(row['tilt_angle']),
                saturated=bool(int(row['saturated']))
            )
    
    return logger.get_data_arrays()


def plot_position_comparison(data1, data2, data3, save_path=None):
    """Plot 1: x(t) and r(t) overlaid for all three methods.
    
    Args:
        data1 (dict): Step PID data
        data2 (dict): One-shot Trajectory data
        data3 (dict): Trajectory Update data
        save_path (str, optional): Path to save plot
    """
    fig, ax = plt.subplots(figsize=(12, 6))
    
    if data1 is not None:
        ax.plot(data1['time'], data1['position_x'], label='Step PID - Position', 
                linewidth=2, color='blue')
        ax.plot(data1['time'], data1['setpoint'], '--', label='Step PID - Setpoint',
                linewidth=1.5, color='blue', alpha=0.7)
    
    if data2 is not None:
        ax.plot(data2['time'], data2['position_x'], label='Traj 1-shot - Position',
                linewidth=2, color='green')
        ax.plot(data2['time'], data2['setpoint'], '--', label='Traj 1-shot - Setpoint',
                linewidth=1.5, color='green', alpha=0.7)
    
    if data3 is not None:
        ax.plot(data3['time'], data3['position_x'], label='Traj Update - Position',
                linewidth=2, color='red')
        ax.plot(data3['time'], data3['setpoint'], '--', label='Traj Update - Setpoint',
                linewidth=1.5, color='red', alpha=0.7)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Position (m)', fontsize=12)
    ax.set_title('Position Tracking: x(t) and r(t)', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"[PLOT] Saved position comparison to {save_path}")
    else:
        plt.show()
    
    plt.close()


def plot_error_comparison(data1, data2, data3, save_path=None):
    """Plot 2: Error e(t) for all methods.
    
    Args:
        data1 (dict): Step PID data
        data2 (dict): One-shot Trajectory data
        data3 (dict): Trajectory Update data
        save_path (str, optional): Path to save plot
    """
    fig, ax = plt.subplots(figsize=(12, 6))
    
    if data1 is not None:
        ax.plot(data1['time'], data1['error'], label='Step PID', 
                linewidth=2, color='blue')
    
    if data2 is not None:
        ax.plot(data2['time'], data2['error'], label='Traj 1-shot',
                linewidth=2, color='green')
    
    if data3 is not None:
        ax.plot(data3['time'], data3['error'], label='Traj Update',
                linewidth=2, color='red')
    
    # Add zero line
    ax.axhline(y=0, color='black', linestyle=':', linewidth=1, alpha=0.5)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Error e(t) (m)', fontsize=12)
    ax.set_title('Tracking Error: e(t) = r(t) - x(t)', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"[PLOT] Saved error comparison to {save_path}")
    else:
        plt.show()
    
    plt.close()


def plot_control_effort(data1, data2, data3, control_limit=15.0, save_path=None):
    """Plot 3: Control effort u(t) with saturation markers.
    
    Args:
        data1 (dict): Step PID data
        data2 (dict): One-shot Trajectory data
        data3 (dict): Trajectory Update data
        control_limit (float): Control signal limit
        save_path (str, optional): Path to save plot
    """
    fig, ax = plt.subplots(figsize=(12, 6))
    
    if data1 is not None:
        # Plot control signal
        ax.plot(data1['time'], data1['control_signal'], label='Step PID', 
                linewidth=2, color='blue')
        # Mark saturation points
        saturated_mask = data1['saturated'] > 0
        if np.any(saturated_mask):
            ax.scatter(data1['time'][saturated_mask], data1['control_signal'][saturated_mask],
                      marker='x', color='blue', s=30, alpha=0.7, label='Step PID (saturated)')
    
    if data2 is not None:
        ax.plot(data2['time'], data2['control_signal'], label='Traj 1-shot',
                linewidth=2, color='green')
        saturated_mask = data2['saturated'] > 0
        if np.any(saturated_mask):
            ax.scatter(data2['time'][saturated_mask], data2['control_signal'][saturated_mask],
                      marker='x', color='green', s=30, alpha=0.7, label='Traj 1-shot (saturated)')
    
    if data3 is not None:
        ax.plot(data3['time'], data3['control_signal'], label='Traj Update',
                linewidth=2, color='red')
        saturated_mask = data3['saturated'] > 0
        if np.any(saturated_mask):
            ax.scatter(data3['time'][saturated_mask], data3['control_signal'][saturated_mask],
                      marker='x', color='red', s=30, alpha=0.7, label='Traj Update (saturated)')
    
    # Add control limits
    ax.axhline(y=control_limit, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Limit')
    ax.axhline(y=-control_limit, color='red', linestyle='--', linewidth=1, alpha=0.5)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Control Signal u(t) (degrees)', fontsize=12)
    ax.set_title('Control Effort: u(t) with Saturation Markers', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"[PLOT] Saved control effort to {save_path}")
    else:
        plt.show()
    
    plt.close()


def plot_smoothness(data1, data2, data3, save_path=None):
    """Plot 4: Jerk/smoothness metric.
    
    Args:
        data1 (dict): Step PID data
        data2 (dict): One-shot Trajectory data
        data3 (dict): Trajectory Update data
        save_path (str, optional): Path to save plot
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot jerk
    if data1 is not None:
        smoothness1 = compute_smoothness(data1['position_x'], data1['time'])
        # Compute jerk over time for plotting
        jerk1 = compute_jerk_over_time(data1['position_x'], data1['time'])
        if jerk1 is not None:
            ax1.plot(jerk1['time'], jerk1['jerk'], label='Step PID', 
                    linewidth=2, color='blue')
    
    if data2 is not None:
        smoothness2 = compute_smoothness(data2['position_x'], data2['time'])
        jerk2 = compute_jerk_over_time(data2['position_x'], data2['time'])
        if jerk2 is not None:
            ax1.plot(jerk2['time'], jerk2['jerk'], label='Traj 1-shot',
                    linewidth=2, color='green')
    
    if data3 is not None:
        smoothness3 = compute_smoothness(data3['position_x'], data3['time'])
        jerk3 = compute_jerk_over_time(data3['position_x'], data3['time'])
        if jerk3 is not None:
            ax1.plot(jerk3['time'], jerk3['jerk'], label='Traj Update',
                    linewidth=2, color='red')
    
    ax1.axhline(y=0, color='black', linestyle=':', linewidth=1, alpha=0.5)
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Jerk (m/s³)', fontsize=12)
    ax1.set_title('Jerk: Derivative of Acceleration', fontsize=14, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(left=0)
    
    # Plot velocity for reference
    if data1 is not None:
        velocity1 = compute_velocity_over_time(data1['position_x'], data1['time'])
        if velocity1 is not None:
            ax2.plot(velocity1['time'], velocity1['velocity'], label='Step PID',
                    linewidth=2, color='blue')
    
    if data2 is not None:
        velocity2 = compute_velocity_over_time(data2['position_x'], data2['time'])
        if velocity2 is not None:
            ax2.plot(velocity2['time'], velocity2['velocity'], label='Traj 1-shot',
                    linewidth=2, color='green')
    
    if data3 is not None:
        velocity3 = compute_velocity_over_time(data3['position_x'], data3['time'])
        if velocity3 is not None:
            ax2.plot(velocity3['time'], velocity3['velocity'], label='Traj Update',
                    linewidth=2, color='red')
    
    ax2.axhline(y=0, color='black', linestyle=':', linewidth=1, alpha=0.5)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Velocity (m/s)', fontsize=12)
    ax2.set_title('Velocity: First Derivative of Position', fontsize=14, fontweight='bold')
    ax2.legend(loc='best', fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(left=0)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"[PLOT] Saved smoothness plot to {save_path}")
    else:
        plt.show()
    
    plt.close()


def compute_jerk_over_time(x_data, t_data):
    """Compute jerk over time for plotting.
    
    Args:
        x_data (np.array): Position data
        t_data (np.array): Time data
        
    Returns:
        dict: {'time': array, 'jerk': array} or None if insufficient data
    """
    if len(x_data) < 3 or len(t_data) < 3:
        return None
    
    # Compute velocity
    dt = np.diff(t_data)
    dt = np.where(dt > 1e-6, dt, 1e-6)
    velocity = np.diff(x_data) / dt
    
    # Compute acceleration
    if len(velocity) < 2:
        return None
    
    dt_accel = dt[:-1]
    dt_accel = np.where(dt_accel > 1e-6, dt_accel, 1e-6)
    acceleration = np.diff(velocity) / dt_accel
    
    # Compute jerk
    if len(acceleration) < 2:
        return None
    
    dt_jerk = dt_accel[:-1]
    dt_jerk = np.where(dt_jerk > 1e-6, dt_jerk, 1e-6)
    jerk = np.diff(acceleration) / dt_jerk
    
    # Time array for jerk (midpoints)
    t_jerk = t_data[2:-1] if len(t_data) > 3 else t_data[1:-1]
    
    return {'time': t_jerk, 'jerk': jerk}


def compute_velocity_over_time(x_data, t_data):
    """Compute velocity over time for plotting.
    
    Args:
        x_data (np.array): Position data
        t_data (np.array): Time data
        
    Returns:
        dict: {'time': array, 'velocity': array} or None if insufficient data
    """
    if len(x_data) < 2 or len(t_data) < 2:
        return None
    
    dt = np.diff(t_data)
    dt = np.where(dt > 1e-6, dt, 1e-6)
    velocity = np.diff(x_data) / dt
    
    # Time array for velocity (midpoints)
    t_vel = (t_data[:-1] + t_data[1:]) / 2
    
    return {'time': t_vel, 'velocity': velocity}


def generate_all_plots(step_pid_file=None, oneshot_file=None, update_file=None):
    """Generate all comparison plots.
    
    Args:
        step_pid_file (str, optional): Path to Step PID CSV file
        oneshot_file (str, optional): Path to One-shot Trajectory CSV file
        update_file (str, optional): Path to Trajectory Update CSV file
    """
    # Find files if not provided
    if step_pid_file is None or oneshot_file is None or update_file is None:
        found_files = find_latest_experiment_files()
        if step_pid_file is None:
            step_pid_file = found_files[0]
        if oneshot_file is None:
            oneshot_file = found_files[1]
        if update_file is None:
            update_file = found_files[2]
    
    # Load data
    print("[PLOT] Loading experiment data...")
    data1 = load_experiment_data(step_pid_file)
    data2 = load_experiment_data(oneshot_file)
    data3 = load_experiment_data(update_file)
    
    if data1 is None and data2 is None and data3 is None:
        print("[ERROR] No experiment data found. Run run_comparison_experiments.py first.")
        return
    
    # Create results directory
    results_dir = "results"
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    # Generate plots
    print("[PLOT] Generating plots...")
    
    plot_position_comparison(
        data1, data2, data3,
        save_path=os.path.join(results_dir, "plot1_position_comparison.png")
    )
    
    plot_error_comparison(
        data1, data2, data3,
        save_path=os.path.join(results_dir, "plot2_error_comparison.png")
    )
    
    plot_control_effort(
        data1, data2, data3,
        control_limit=15.0,
        save_path=os.path.join(results_dir, "plot3_control_effort.png")
    )
    
    plot_smoothness(
        data1, data2, data3,
        save_path=os.path.join(results_dir, "plot4_smoothness.png")
    )
    
    print("[PLOT] All plots generated successfully!")


def main():
    """Main function to generate all plots."""
    print("\n" + "="*60)
    print("GENERATING COMPARISON PLOTS")
    print("="*60)
    
    generate_all_plots()


if __name__ == "__main__":
    main()

