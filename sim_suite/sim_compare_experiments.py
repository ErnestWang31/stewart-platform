# Experiment Comparison Tool (works with both real and simulated experiments)
# Loads CSV data from three experiments, computes metrics, and generates visualizations

import sys
import os
import csv
import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import glob
from datetime import datetime

# Add parent directory to path to import existing modules
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from metrics import compute_all_metrics, print_metrics

class ExperimentComparator:
    """Compares results from Step PID, One-shot Trajectory, and Trajectory Update experiments."""
    
    def __init__(self, results_dir="results"):
        """Initialize comparator.
        
        Args:
            results_dir: Directory containing experiment CSV files
        """
        self.results_dir = Path(results_dir)
        self.experiments = {}
        
    def load_experiment(self, filepath, method_name):
        """Load experiment data from CSV file.
        
        Args:
            filepath: Path to CSV file
            method_name: Name of the method (e.g., "Step PID")
            
        Returns:
            dict: Experiment data and metadata
        """
        # Read CSV file
        time_data = []
        position_data = []
        setpoint_data = []
        error_data = []
        control_data = []
        tilt_data = []
        saturated_data = []
        
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                time_data.append(float(row['time']))
                position_data.append(float(row['position_x']))
                setpoint_data.append(float(row['setpoint']))
                error_data.append(float(row['error']))
                control_data.append(float(row['control_signal']))
                tilt_data.append(float(row['tilt_angle']))
                saturated_data.append(int(row['saturated']))
        
        # Convert to numpy arrays
        data = {
            'time': np.array(time_data),
            'position': np.array(position_data),
            'setpoint': np.array(setpoint_data),
            'error': np.array(error_data),
            'control': np.array(control_data),
            'tilt': np.array(tilt_data),
            'saturated': np.array(saturated_data),
            'method': method_name,
            'filename': Path(filepath).name
        }
        
        # Try to load trajectory segments for trajectory update experiments
        filepath_str = str(filepath)
        if 'trajectory_update' in filepath_str.lower():
            segments_path = filepath_str.replace('.csv', '_segments.json')
            if Path(segments_path).exists():
                try:
                    with open(segments_path, 'r') as f:
                        data['trajectory_segments'] = json.load(f)
                    print(f"[LOAD] Loaded {len(data['trajectory_segments'])} trajectory segments")
                except Exception as e:
                    print(f"[WARNING] Could not load trajectory segments: {e}")
                    data['trajectory_segments'] = []
            else:
                data['trajectory_segments'] = []
        
        return data
    
    def find_latest_experiments(self):
        """Find the most recent experiment files for each method.
        
        Returns:
            dict: Mapping of method names to file paths
        """
        files = {}
        
        # Find latest step PID
        step_pid_files = list(self.results_dir.glob("experiment_step_pid_*.csv"))
        if step_pid_files:
            files['Step PID'] = max(step_pid_files, key=lambda p: p.stat().st_mtime)
        
        # Find latest one-shot trajectory
        oneshot_files = list(self.results_dir.glob("experiment_oneshot_trajectory_*.csv"))
        if oneshot_files:
            files['One-Shot Trajectory'] = max(oneshot_files, key=lambda p: p.stat().st_mtime)
        
        # Find latest trajectory update
        update_files = list(self.results_dir.glob("experiment_trajectory_update_*.csv"))
        if update_files:
            files['Trajectory Update'] = max(update_files, key=lambda p: p.stat().st_mtime)
        
        return files
    
    def load_all_experiments(self, file_dict=None):
        """Load all three experiment types.
        
        Args:
            file_dict: Optional dict mapping method names to file paths.
                      If None, auto-detects latest files.
        """
        if file_dict is None:
            file_dict = self.find_latest_experiments()
        
        for method_name, filepath in file_dict.items():
            if filepath and Path(filepath).exists():
                print(f"[LOAD] Loading {method_name} from {Path(filepath).name}")
                self.experiments[method_name] = self.load_experiment(filepath, method_name)
            else:
                print(f"[WARNING] File not found for {method_name}: {filepath}")
    
    def compute_all_metrics(self):
        """Compute metrics for all loaded experiments.
        
        Returns:
            dict: Mapping of method names to metrics dictionaries
        """
        all_metrics = {}
        
        for method_name, data in self.experiments.items():
            print(f"\n[COMPUTE] Computing metrics for {method_name}...")
            metrics = compute_all_metrics(
                data['time'],
                data['position'],
                data['setpoint'],
                data['error'],
                data['control'],
                data['tilt'],
                data['saturated'],
                tolerance=0.005,  # 5mm
                settle_duration=0.5
            )
            all_metrics[method_name] = metrics
            print_metrics(metrics, method_name)
        
        return all_metrics
    
    def plot_comparison(self, save_path=None):
        """Generate comparison plots.
        
        Args:
            save_path: Optional path to save plots. If None, displays interactively.
        """
        if len(self.experiments) == 0:
            print("[ERROR] No experiments loaded. Load experiments first.")
            return
        
        # Create figure with 4 subplots
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)
        
        # Color scheme for each method
        colors = {
            'Step PID': '#1f77b4',  # Blue
            'One-Shot Trajectory': '#ff7f0e',  # Orange
            'Trajectory Update': '#2ca02c'  # Green
        }
        
        # Plot 1: Position and Setpoint overlaid
        ax1 = fig.add_subplot(gs[0, 0])
        for method_name, data in self.experiments.items():
            color = colors.get(method_name, 'gray')
            ax1.plot(data['time'], data['position'] * 100, 
                    label=f"{method_name} - Position", 
                    color=color, linewidth=2, alpha=0.8)
            
            # For trajectory update, plot individual trajectory segments
            if 'trajectory_segments' in data and len(data['trajectory_segments']) > 0:
                # Plot each trajectory segment as a separate line
                for i, seg in enumerate(data['trajectory_segments']):
                    x0 = seg['x0'] * 100  # Convert to cm
                    xf = seg['xf'] * 100
                    t_start = seg['start_time']
                    duration = seg['duration']
                    t_end = t_start + duration
                    
                    # Generate points for this trajectory segment
                    if seg['method'] == 'linear':
                        # Linear trajectory: r(t) = x0 + (xf - x0) * (t/T)
                        t_seg = np.linspace(t_start, t_end, 50)
                        r_seg = x0 + (xf - x0) * ((t_seg - t_start) / duration) if duration > 0 else np.full_like(t_seg, xf)
                    else:
                        # Polynomial trajectory
                        t_seg = np.linspace(t_start, t_end, 50)
                        tau = (t_seg - t_start) / duration if duration > 0 else np.ones_like(t_seg)
                        poly = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
                        r_seg = x0 + (xf - x0) * poly
                    
                    # Plot trajectory segment (lighter, thinner line)
                    ax1.plot(t_seg, r_seg, 
                            color=color, linestyle='--', linewidth=1, alpha=0.4,
                            label='Trajectory segments' if i == 0 else '')
            else:
                # For other methods, plot setpoint as normal
                ax1.plot(data['time'], data['setpoint'] * 100,
                        label=f"{method_name} - Setpoint",
                        color=color, linestyle='--', linewidth=1.5, alpha=0.6)
        
        ax1.axhline(0, color='black', linestyle=':', alpha=0.3, linewidth=1)
        ax1.set_xlabel('Time (s)', fontsize=11)
        ax1.set_ylabel('Position (cm)', fontsize=11)
        ax1.set_title('Position vs Setpoint', fontsize=12, fontweight='bold')
        ax1.legend(loc='best', fontsize=9)
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Error over time
        ax2 = fig.add_subplot(gs[0, 1])
        for method_name, data in self.experiments.items():
            color = colors.get(method_name, 'gray')
            ax2.plot(data['time'], data['error'] * 100,
                    label=method_name, color=color, linewidth=2, alpha=0.8)
        ax2.axhline(0, color='black', linestyle=':', alpha=0.3, linewidth=1)
        ax2.axhline(0.5, color='red', linestyle='--', alpha=0.5, linewidth=1, label='Tolerance (±5mm)')
        ax2.axhline(-0.5, color='red', linestyle='--', alpha=0.5, linewidth=1)
        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Error (cm)', fontsize=11)
        ax2.set_title('Tracking Error', fontsize=12, fontweight='bold')
        ax2.legend(loc='best', fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Control effort with saturation markers
        ax3 = fig.add_subplot(gs[1, 0])
        for method_name, data in self.experiments.items():
            color = colors.get(method_name, 'gray')
            # Plot control signal
            ax3.plot(data['time'], data['control'],
                    label=method_name, color=color, linewidth=2, alpha=0.8)
            # Mark saturation points
            saturated_mask = data['saturated'] == 1
            if np.any(saturated_mask):
                ax3.scatter(data['time'][saturated_mask], data['control'][saturated_mask],
                           color=color, marker='x', s=50, alpha=0.7, 
                           label=f"{method_name} (saturated)", zorder=5)
        ax3.axhline(30, color='red', linestyle='--', alpha=0.5, linewidth=1, label='Limit (±30°)')
        ax3.axhline(-30, color='red', linestyle='--', alpha=0.5, linewidth=1)
        ax3.axhline(0, color='black', linestyle=':', alpha=0.3, linewidth=1)
        ax3.set_xlabel('Time (s)', fontsize=11)
        ax3.set_ylabel('Control Signal (degrees)', fontsize=11)
        ax3.set_title('Control Effort with Saturation Markers', fontsize=12, fontweight='bold')
        ax3.legend(loc='best', fontsize=9)
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Jerk (smoothness metric)
        ax4 = fig.add_subplot(gs[1, 1])
        for method_name, data in self.experiments.items():
            color = colors.get(method_name, 'gray')
            # Calculate jerk
            time_arr = data['time']
            pos_arr = data['position']
            
            if len(time_arr) >= 3:
                # Calculate velocity
                dt = np.diff(time_arr)
                dt = np.where(dt > 1e-6, dt, 1e-6)
                velocity = np.diff(pos_arr) / dt
                
                if len(velocity) >= 2:
                    # Calculate acceleration
                    dt_vel = np.diff(time_arr[1:])
                    dt_vel = np.where(dt_vel > 1e-6, dt_vel, 1e-6)
                    acceleration = np.diff(velocity) / dt_vel
                    
                    if len(acceleration) >= 1:
                        # Calculate jerk
                        dt_acc = np.diff(time_arr[2:])
                        dt_acc = np.where(dt_acc > 1e-6, dt_acc, 1e-6)
                        jerk = np.diff(acceleration) / dt_acc
                        jerk_time = time_arr[3:]
                        
                        ax4.plot(jerk_time, jerk,
                                label=method_name, color=color, linewidth=2, alpha=0.8)
        
        ax4.axhline(0, color='black', linestyle=':', alpha=0.3, linewidth=1)
        ax4.set_xlabel('Time (s)', fontsize=11)
        ax4.set_ylabel('Jerk (m/s³)', fontsize=11)
        ax4.set_title('Jerk (Smoothness Metric)', fontsize=12, fontweight='bold')
        ax4.legend(loc='best', fontsize=9)
        ax4.grid(True, alpha=0.3)
        
        # Overall title
        fig.suptitle('Controller Comparison: Step PID vs One-Shot Trajectory vs Trajectory Update',
                    fontsize=14, fontweight='bold', y=0.995)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"\n[SAVE] Plots saved to {save_path}")
        
        # Always show the plot
        plt.show()
        
        return fig
    
    def generate_comparison_table(self, metrics_dict, save_path=None):
        """Generate comparison table in Markdown and CSV formats.
        
        Args:
            metrics_dict: Dictionary mapping method names to metrics
            save_path: Optional base path for saving (without extension)
        """
        # Prepare table data
        table_data = []
        for method_name in ['Step PID', 'One-Shot Trajectory', 'Trajectory Update']:
            if method_name in metrics_dict:
                m = metrics_dict[method_name]
                completion = f"{m['completion_time']:.3f}" if m['completion_time'] is not None else "N/A"
                table_data.append({
                    'Method': method_name,
                    'Completion Time (s)': completion,
                    'RMSE (m)': f"{m['rmse']:.6f}",
                    'Max Deviation (m)': f"{m['max_deviation']:.6f}",
                    'Max Control (°)': f"{m['max_control']:.3f}",
                    'Saturation %': f"{m['saturation_pct']:.1f}%",
                    'Max Jerk (m/s³)': f"{m['max_jerk']:.3f}",
                    'RMS Jerk (m/s³)': f"{m['rms_jerk']:.3f}",
                    'Vel. Reversals': f"{m['velocity_reversals']}"
                })
        
        # Generate Markdown table
        md_lines = [
            "# Controller Comparison Results",
            "",
            "## Comparison Table",
            "",
            "| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |",
            "|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|"
        ]
        
        for row in table_data:
            md_lines.append(
                f"| {row['Method']} | {row['Completion Time (s)']} | {row['RMSE (m)']} | "
                f"{row['Max Deviation (m)']} | {row['Max Control (°)']} | {row['Saturation %']} | "
                f"{row['Max Jerk (m/s³)']} | {row['RMS Jerk (m/s³)']} | {row['Vel. Reversals']} |"
            )
        
        md_lines.extend([
            "",
            "## Detailed Metrics",
            ""
        ])
        
        for method_name in ['Step PID', 'One-Shot Trajectory', 'Trajectory Update']:
            if method_name in metrics_dict:
                m = metrics_dict[method_name]
                completion_str = f"{m['completion_time']:.3f}s" if m['completion_time'] is not None else 'N/A'
                md_lines.extend([
                    f"### {method_name}",
                    f"- Completion Time: {completion_str}",
                    f"- RMSE: {m['rmse']:.6f} m",
                    f"- Max Deviation: {m['max_deviation']:.6f} m",
                    f"- Max Control: {m['max_control']:.3f}°",
                    f"- Saturation: {m['saturation_pct']:.1f}%",
                    f"- Max Jerk: {m['max_jerk']:.3f} m/s³",
                    f"- RMS Jerk: {m['rms_jerk']:.3f} m/s³",
                    f"- Velocity Reversals: {m['velocity_reversals']}",
                    ""
                ])
        
        md_text = "\n".join(md_lines)
        
        # Save Markdown
        if save_path:
            md_file = f"{save_path}.md"
            with open(md_file, 'w') as f:
                f.write(md_text)
            print(f"[SAVE] Comparison table saved to {md_file}")
        
        # Save CSV
        if save_path:
            csv_file = f"{save_path}.csv"
            with open(csv_file, 'w', newline='') as f:
                if table_data:
                    writer = csv.DictWriter(f, fieldnames=table_data[0].keys())
                    writer.writeheader()
                    writer.writerows(table_data)
            print(f"[SAVE] Comparison table (CSV) saved to {csv_file}")
        
        # Print to console
        print("\n" + "="*100)
        print("COMPARISON TABLE")
        print("="*100)
        print(md_text)
        
        return md_text
    
    def run_comparison(self, file_dict=None, save_plots=True, save_table=True):
        """Run full comparison analysis.
        
        Args:
            file_dict: Optional dict mapping method names to file paths
            save_plots: Whether to save plots
            save_table: Whether to save comparison table
        """
        print("="*80)
        print("EXPERIMENT COMPARISON TOOL")
        print("="*80)
        
        # Load experiments
        print("\n[STEP 1] Loading experiment data...")
        self.load_all_experiments(file_dict)
        
        if len(self.experiments) == 0:
            print("[ERROR] No experiments loaded. Check file paths.")
            return
        
        # Compute metrics
        print("\n[STEP 2] Computing metrics...")
        metrics_dict = self.compute_all_metrics()
        
        # Generate plots
        print("\n[STEP 3] Generating comparison plots...")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_path = self.results_dir / f"comparison_plots_{timestamp}.png" if save_plots else None
        self.plot_comparison(plot_path)
        
        # Generate comparison table
        print("\n[STEP 4] Generating comparison table...")
        table_path = self.results_dir / f"comparison_table_{timestamp}" if save_table else None
        self.generate_comparison_table(metrics_dict, table_path)
        
        print("\n" + "="*80)
        print("COMPARISON COMPLETE!")
        print("="*80)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Compare experiment results from three control methods')
    parser.add_argument('--step-pid', type=str, help='Path to Step PID experiment CSV file')
    parser.add_argument('--oneshot', type=str, help='Path to One-Shot Trajectory experiment CSV file')
    parser.add_argument('--update', type=str, help='Path to Trajectory Update experiment CSV file')
    parser.add_argument('--no-save', action='store_true', help='Do not save plots and tables (display only)')
    parser.add_argument('--results-dir', type=str, default='results', help='Results directory (default: results)')
    
    args = parser.parse_args()
    
    comparator = ExperimentComparator(results_dir=args.results_dir)
    
    # Build file dictionary if files specified
    file_dict = None
    if args.step_pid or args.oneshot or args.update:
        file_dict = {}
        if args.step_pid:
            file_dict['Step PID'] = args.step_pid
        if args.oneshot:
            file_dict['One-Shot Trajectory'] = args.oneshot
        if args.update:
            file_dict['Trajectory Update'] = args.update
    
    comparator.run_comparison(
        file_dict=file_dict,
        save_plots=not args.no_save,
        save_table=not args.no_save
    )
