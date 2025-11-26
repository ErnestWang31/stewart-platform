"""
Analyze comparison table data using the same metrics framework
"""
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def analyze_comparison_table():
    """Analyze the comparison table data"""
    results_dir = Path(__file__).parent / 'results'
    csv_file = results_dir / 'comparison_table_20251125_223231.csv'
    
    # Try different encodings
    encodings = ['utf-8', 'latin-1', 'cp1252', 'iso-8859-1']
    df = None
    for enc in encodings:
        try:
            df = pd.read_csv(csv_file, encoding=enc)
            break
        except:
            continue
    
    if df is None:
        raise ValueError(f"Could not read {csv_file} with any encoding")
    
    # Clean up column names - handle encoding issues
    df.columns = df.columns.str.strip()
    
    # Map column names (handle encoding issues)
    col_mapping = {}
    for col in df.columns:
        if 'Completion Time' in col:
            col_mapping[col] = 'completion_time'
        elif 'RMSE' in col:
            col_mapping[col] = 'rmse'
        elif 'Max Deviation' in col:
            col_mapping[col] = 'max_deviation'
        elif 'Max Control' in col:
            col_mapping[col] = 'max_control'
        elif 'Saturation' in col:
            col_mapping[col] = 'saturation'
        elif 'Max Jerk' in col:
            col_mapping[col] = 'max_jerk'
        elif 'RMS Jerk' in col:
            col_mapping[col] = 'rms_jerk'
        elif 'Vel. Reversals' in col or 'Reversals' in col:
            col_mapping[col] = 'velocity_reversals'
    
    # Rename columns
    df = df.rename(columns=col_mapping)
    
    print("="*70)
    print("COMPARISON TABLE ANALYSIS")
    print("="*70)
    print(f"\nData from: {csv_file.name}\n")
    
    # Display the data
    print("Raw Data:")
    print(df.to_string(index=False))
    print("\n" + "="*70)
    
    # Extract metrics for each method
    methods = df['Method'].values
    metrics = {}
    
    for idx, method in enumerate(methods):
        sat_str = str(df.iloc[idx]['saturation']).replace('%', '').strip()
        metrics[method] = {
            'completion_time': float(df.iloc[idx]['completion_time']),
            'rmse': float(df.iloc[idx]['rmse']),
            'max_deviation': float(df.iloc[idx]['max_deviation']),
            'max_control_signal': float(df.iloc[idx]['max_control']),
            'saturation_percentage': float(sat_str) if sat_str else 0.0,
            'max_jerk': float(df.iloc[idx]['max_jerk']),
            'rms_jerk': float(df.iloc[idx]['rms_jerk']),
            'velocity_reversals': int(df.iloc[idx]['velocity_reversals'])
        }
    
    # Print detailed analysis
    print("\nDETAILED METRICS ANALYSIS")
    print("="*70)
    
    for method in methods:
        m = metrics[method]
        print(f"\n{method}:")
        print(f"  Completion Time (5mm tolerance): {m['completion_time']:.3f} s")
        print(f"  RMSE: {m['rmse']:.6f} m")
        print(f"  Max Deviation: {m['max_deviation']:.6f} m")
        print(f"  Max Control Signal: {m['max_control_signal']:.3f}")
        print(f"  Saturation: {m['saturation_percentage']:.2f}%")
        print(f"  Max Jerk: {m['max_jerk']:.3f} m/s³")
        print(f"  RMS Jerk: {m['rms_jerk']:.3f} m/s³")
        print(f"  Velocity Reversals: {m['velocity_reversals']}")
    
    # Try to load SSE data
    sse_file = results_dir / 'comparison_table_with_sse.csv'
    sse_data = {}
    if sse_file.exists():
        try:
            df_sse = pd.read_csv(sse_file, encoding='latin-1')
            for idx, row in df_sse.iterrows():
                method = row['Method']
                if 'SSE Mean (m)' in df_sse.columns:
                    sse_data[method] = {
                        'sse_mean': float(row['SSE Mean (m)']),
                        'sse_std': float(row['SSE Std (m)']),
                        'sse_rms': float(row['SSE RMS (m)'])
                    }
        except:
            pass
    
    # Find best performers
    print("\n" + "="*70)
    print("BEST PERFORMERS BY METRIC")
    print("="*70)
    
    # Completion time (lower is better)
    best_time = min(metrics.items(), key=lambda x: x[1]['completion_time'])
    print(f"Fastest Completion: {best_time[0]} ({best_time[1]['completion_time']:.3f} s)")
    
    # RMSE (lower is better)
    best_rmse = min(metrics.items(), key=lambda x: x[1]['rmse'])
    print(f"Lowest RMSE: {best_rmse[0]} ({best_rmse[1]['rmse']:.6f} m)")
    
    # Max deviation (lower is better)
    best_dev = min(metrics.items(), key=lambda x: x[1]['max_deviation'])
    print(f"Smallest Max Deviation: {best_dev[0]} ({best_dev[1]['max_deviation']:.6f} m)")
    
    # Control effort (lower is better)
    best_control = min(metrics.items(), key=lambda x: x[1]['max_control_signal'])
    print(f"Lowest Control Effort: {best_control[0]} ({best_control[1]['max_control_signal']:.3f})")
    
    # Smoothness - jerk (lower is better)
    best_jerk = min(metrics.items(), key=lambda x: x[1]['max_jerk'])
    print(f"Lowest Max Jerk: {best_jerk[0]} ({best_jerk[1]['max_jerk']:.3f} m/s³)")
    
    best_rms_jerk = min(metrics.items(), key=lambda x: x[1]['rms_jerk'])
    print(f"Lowest RMS Jerk: {best_rms_jerk[0]} ({best_rms_jerk[1]['rms_jerk']:.3f} m/s³)")
    
    # Velocity reversals (lower is better)
    best_reversals = min(metrics.items(), key=lambda x: x[1]['velocity_reversals'])
    print(f"Fewest Velocity Reversals: {best_reversals[0]} ({best_reversals[1]['velocity_reversals']})")
    
    # SSE (if available)
    if sse_data:
        best_sse = min(sse_data.items(), key=lambda x: x[1]['sse_mean'])
        print(f"Best SSE (lowest mean): {best_sse[0]} ({best_sse[1]['sse_mean']:.6f} m)")
    
    # Try to load SSE data if available
    sse_file = results_dir / 'comparison_table_with_sse.csv'
    sse_data = {}
    if sse_file.exists():
        try:
            df_sse = pd.read_csv(sse_file, encoding='latin-1')
            for idx, row in df_sse.iterrows():
                method = row['Method']
                if 'SSE Mean (m)' in df_sse.columns:
                    sse_data[method] = {
                        'sse_mean': float(row['SSE Mean (m)']),
                        'sse_std': float(row['SSE Std (m)']),
                        'sse_rms': float(row['SSE RMS (m)'])
                    }
        except:
            pass
    
    # Create comparison visualization
    if sse_data:
        fig, axes = plt.subplots(3, 3, figsize=(18, 12))
        metrics_to_plot = [
            ('completion_time', 'Completion Time (s)', axes[0, 0], 'lower'),
            ('rmse', 'RMSE (m)', axes[0, 1], 'lower'),
            ('max_deviation', 'Max Deviation (m)', axes[0, 2], 'lower'),
            ('max_control_signal', 'Max Control Signal', axes[1, 0], 'lower'),
            ('max_jerk', 'Max Jerk (m/s³)', axes[1, 1], 'lower'),
            ('rms_jerk', 'RMS Jerk (m/s³)', axes[1, 2], 'lower'),
            ('velocity_reversals', 'Velocity Reversals', axes[2, 0], 'lower'),
            ('saturation_percentage', 'Saturation %', axes[2, 1], 'lower'),
        ]
        # Add SSE plot
        if sse_data:
            axes_sse = axes[2, 2]
            sse_values = [sse_data.get(m, {}).get('sse_mean', 0) for m in methods]
            bars = axes_sse.bar(methods, sse_values, color=['#1f77b4', '#ff7f0e', '#2ca02c'])
            axes_sse.set_ylabel('SSE Mean (m)')
            axes_sse.set_title('SSE Mean (m)')
            axes_sse.tick_params(axis='x', rotation=45)
            axes_sse.grid(axis='y', alpha=0.3)
            for bar in bars:
                height = bar.get_height()
                axes_sse.text(bar.get_x() + bar.get_width()/2., height,
                           f'{height:.4f}',
                           ha='center', va='bottom', fontsize=9)
    else:
        fig, axes = plt.subplots(2, 4, figsize=(16, 10))
        metrics_to_plot = [
            ('completion_time', 'Completion Time (s)', axes[0, 0], 'lower'),
            ('rmse', 'RMSE (m)', axes[0, 1], 'lower'),
            ('max_deviation', 'Max Deviation (m)', axes[0, 2], 'lower'),
            ('max_control_signal', 'Max Control Signal', axes[0, 3], 'lower'),
            ('max_jerk', 'Max Jerk (m/s³)', axes[1, 0], 'lower'),
            ('rms_jerk', 'RMS Jerk (m/s³)', axes[1, 1], 'lower'),
            ('velocity_reversals', 'Velocity Reversals', axes[1, 2], 'lower'),
            ('saturation_percentage', 'Saturation %', axes[1, 3], 'lower'),
        ]
    
    fig.suptitle('Experiment Metrics Comparison (New Data)', fontsize=16, fontweight='bold')
    
    for metric_key, ylabel, ax, better in metrics_to_plot:
        values = [metrics[method][metric_key] for method in methods]
        
        bars = ax.bar(methods, values, color=['#1f77b4', '#ff7f0e', '#2ca02c'])
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)
        ax.tick_params(axis='x', rotation=45)
        ax.grid(axis='y', alpha=0.3)
        
        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            if metric_key == 'saturation_percentage':
                label = f'{height:.1f}%'
            elif metric_key in ['completion_time', 'rmse', 'max_deviation', 'max_control_signal', 'max_jerk', 'rms_jerk']:
                label = f'{height:.3f}'
            else:
                label = f'{int(height)}'
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   label,
                   ha='center', va='bottom', fontsize=9)
    
    plt.tight_layout()
    plot_file = results_dir / 'comparison_table_analysis.png'
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {plot_file}")
    plt.close()
    
    # Create summary comparison table
    print("\n" + "="*70)
    print("SUMMARY COMPARISON")
    print("="*70)
    
    summary_df = pd.DataFrame(metrics).T
    summary_df = summary_df.round(3)
    print("\n" + summary_df.to_string())
    
    # Save summary
    summary_file = results_dir / 'comparison_table_summary.csv'
    summary_df.to_csv(summary_file)
    print(f"\nSummary saved to: {summary_file}")
    
    return metrics

if __name__ == '__main__':
    analyze_comparison_table()

