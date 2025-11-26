"""
Analyze experiment data for metrics specified in TEST_PLAN.md (lines 23-28)
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def calculate_metrics(df, experiment_name):
    """Calculate all metrics for an experiment"""
    metrics = {}
    
    # Convert error to absolute value for tolerance checking
    abs_error = np.abs(df['error'])
    tolerance = 0.005  # 5mm tolerance
    
    # 1. Completion time (time to reach and stay within 5mm tolerance of FINAL setpoint)
    # For trajectory experiments, we want time to reach the final target, not intermediate waypoints
    final_setpoint = df['setpoint'].iloc[-1]
    
    # Calculate error relative to final setpoint
    error_to_final = df['position_x'] - final_setpoint
    abs_error_to_final = np.abs(error_to_final)
    
    # Find first time when error to final setpoint stays within tolerance for at least 0.1 seconds
    within_tolerance = abs_error_to_final < tolerance
    completion_time = None
    window_size = int(0.1 / (df['time'].iloc[1] - df['time'].iloc[0])) if len(df) > 1 else 10
    
    for i in range(len(df) - window_size):
        if within_tolerance.iloc[i:i+window_size].all():
            completion_time = df['time'].iloc[i]
            break
    
    metrics['completion_time'] = completion_time if completion_time is not None else df['time'].iloc[-1]
    metrics['final_setpoint'] = final_setpoint
    
    # 2. RMSE (Root Mean Square Error)
    metrics['rmse'] = np.sqrt(np.mean(df['error']**2))
    
    # 3. Max overshoot/deviation
    # Max overshoot is the maximum absolute error after reaching the setpoint
    if df['setpoint'].iloc[0] != 0:
        # For non-zero setpoints, find when we first cross the setpoint
        initial_error = abs(df['error'].iloc[0])
        metrics['max_overshoot'] = np.max(np.abs(df['error'])) - initial_error if np.max(np.abs(df['error'])) > initial_error else 0
    else:
        # For step response, max overshoot is max absolute error
        metrics['max_overshoot'] = np.max(np.abs(df['error']))
    
    metrics['max_deviation'] = np.max(np.abs(df['error']))
    
    # 4. Control effort (max control signal, saturation %)
    metrics['max_control_signal'] = np.max(np.abs(df['control_signal']))
    metrics['saturation_percentage'] = (df['saturated'].sum() / len(df)) * 100
    
    # 5. Smoothness (jerk, velocity reversals)
    # Jerk is the derivative of acceleration (third derivative of position)
    if len(df) > 3:
        # Calculate velocity (first derivative of position)
        velocity = np.diff(df['position_x']) / np.diff(df['time'])
        # Calculate acceleration (second derivative)
        if len(velocity) > 1:
            acceleration = np.diff(velocity) / np.diff(df['time'][1:])
            # Calculate jerk (third derivative)
            if len(acceleration) > 1:
                jerk = np.diff(acceleration) / np.diff(df['time'][2:])
                metrics['max_jerk'] = np.max(np.abs(jerk))
                metrics['mean_abs_jerk'] = np.mean(np.abs(jerk))
            else:
                metrics['max_jerk'] = 0
                metrics['mean_abs_jerk'] = 0
            
            # Velocity reversals: count sign changes in velocity
            velocity_sign_changes = np.sum(np.diff(np.sign(velocity)) != 0)
            metrics['velocity_reversals'] = velocity_sign_changes
        else:
            metrics['max_jerk'] = 0
            metrics['mean_abs_jerk'] = 0
            metrics['velocity_reversals'] = 0
    else:
        metrics['max_jerk'] = 0
        metrics['mean_abs_jerk'] = 0
        metrics['velocity_reversals'] = 0
    
    # 6. Steady-state error
    # Use last 20% of data to calculate steady-state error
    steady_state_start = int(len(df) * 0.8)
    steady_state_errors = df['error'].iloc[steady_state_start:]
    metrics['steady_state_error_mean'] = np.mean(np.abs(steady_state_errors))
    metrics['steady_state_error_std'] = np.std(steady_state_errors)
    
    return metrics

def main():
    results_dir = Path(__file__).parent / 'results'
    
    # Load all three experiments
    experiments = {
        'oneshot_trajectory': 'experiment_oneshot_trajectory_20251125_212903.csv',
        'trajectory_update': 'experiment_trajectory_update_20251125_212946.csv',
        'step_pid': 'experiment_step_pid_20251125_213037.csv'
    }
    
    all_metrics = {}
    
    for exp_name, filename in experiments.items():
        filepath = results_dir / filename
        if not filepath.exists():
            print(f"Warning: {filename} not found")
            continue
        
        df = pd.read_csv(filepath)
        metrics = calculate_metrics(df, exp_name)
        all_metrics[exp_name] = metrics
        
        print(f"\n{'='*60}")
        print(f"Experiment: {exp_name}")
        print(f"{'='*60}")
        print(f"Initial setpoint: {df['setpoint'].iloc[0]:.6f} m")
        print(f"Final setpoint: {metrics['final_setpoint']:.6f} m")
        print(f"Completion time (5mm tolerance to final setpoint): {metrics['completion_time']:.3f} s")
        print(f"RMSE: {metrics['rmse']:.6f} m")
        print(f"Max overshoot: {metrics['max_overshoot']:.6f} m")
        print(f"Max deviation: {metrics['max_deviation']:.6f} m")
        print(f"Max control signal: {metrics['max_control_signal']:.3f}")
        print(f"Saturation percentage: {metrics['saturation_percentage']:.2f}%")
        print(f"Max jerk: {metrics['max_jerk']:.3f} m/s³")
        print(f"Mean absolute jerk: {metrics['mean_abs_jerk']:.3f} m/s³")
        print(f"Velocity reversals: {metrics['velocity_reversals']}")
        print(f"Steady-state error (mean): {metrics['steady_state_error_mean']:.6f} m")
        print(f"Steady-state error (std): {metrics['steady_state_error_std']:.6f} m")
    
    # Create comparison table
    print(f"\n{'='*60}")
    print("COMPARISON TABLE")
    print(f"{'='*60}")
    
    # Create DataFrame for comparison
    comparison_df = pd.DataFrame(all_metrics).T
    comparison_df = comparison_df.round(6)
    
    print("\n" + comparison_df.to_string())
    
    # Save comparison to CSV
    output_file = results_dir / 'experiment_metrics_comparison.csv'
    comparison_df.to_csv(output_file)
    print(f"\nComparison saved to: {output_file}")
    
    # Create visualization
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Experiment Metrics Comparison', fontsize=16)
    
    metrics_to_plot = [
        ('completion_time', 'Completion Time (s)', axes[0, 0]),
        ('rmse', 'RMSE (m)', axes[0, 1]),
        ('max_overshoot', 'Max Overshoot (m)', axes[0, 2]),
        ('max_control_signal', 'Max Control Signal', axes[1, 0]),
        ('saturation_percentage', 'Saturation %', axes[1, 1]),
        ('steady_state_error_mean', 'Steady-State Error (m)', axes[1, 2])
    ]
    
    for metric_key, ylabel, ax in metrics_to_plot:
        values = [all_metrics[exp][metric_key] for exp in experiments.keys() if exp in all_metrics]
        labels = [exp for exp in experiments.keys() if exp in all_metrics]
        
        bars = ax.bar(labels, values)
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)
        ax.tick_params(axis='x', rotation=45)
        
        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{height:.4f}',
                   ha='center', va='bottom', fontsize=8)
    
    plt.tight_layout()
    plot_file = results_dir / 'experiment_metrics_comparison.png'
    plt.savefig(plot_file, dpi=150)
    print(f"Plot saved to: {plot_file}")
    plt.close()

if __name__ == '__main__':
    main()

