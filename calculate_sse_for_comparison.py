"""
Calculate Steady-State Error (SSE) for the comparison table experiments
"""
import pandas as pd
import numpy as np
from pathlib import Path

def calculate_sse(df):
    """Calculate steady-state error using last 20% of data"""
    steady_state_start = int(len(df) * 0.8)
    steady_state_errors = df['error'].iloc[steady_state_start:]
    
    sse_mean = np.mean(np.abs(steady_state_errors))
    sse_std = np.std(steady_state_errors)
    sse_rms = np.sqrt(np.mean(steady_state_errors**2))
    
    return {
        'sse_mean': sse_mean,
        'sse_std': sse_std,
        'sse_rms': sse_rms
    }

def main():
    results_dir = Path(__file__).parent / 'results'
    
    # Find experiment files - try to find files closest to comparison table timestamp
    # The comparison table is from 20251125_223231, so look for files around that time
    step_pid_files = list(results_dir.glob("experiment_step_pid_*.csv"))
    oneshot_files = list(results_dir.glob("experiment_oneshot_trajectory_*.csv"))
    trajectory_update_files = list(results_dir.glob("experiment_trajectory_update_*.csv"))
    
    if not step_pid_files or not oneshot_files or not trajectory_update_files:
        print("Error: Could not find all three experiment files")
        return
    
    # Get latest files (most likely to match comparison table)
    latest_step_pid = max(step_pid_files, key=lambda p: p.stat().st_mtime)
    latest_oneshot = max(oneshot_files, key=lambda p: p.stat().st_mtime)
    latest_trajectory_update = max(trajectory_update_files, key=lambda p: p.stat().st_mtime)
    
    print("="*70)
    print("STEADY-STATE ERROR CALCULATION")
    print("="*70)
    print(f"\nFiles analyzed:")
    print(f"  Step PID: {latest_step_pid.name}")
    print(f"  One-Shot Trajectory: {latest_oneshot.name}")
    print(f"  Trajectory Update: {latest_trajectory_update.name}")
    
    experiments = {
        'Step PID': latest_step_pid,
        'One-Shot Trajectory': latest_oneshot,
        'Trajectory Update': latest_trajectory_update
    }
    
    sse_results = {}
    
    for method, filepath in experiments.items():
        df = pd.read_csv(filepath)
        sse = calculate_sse(df)
        sse_results[method] = sse
        
        print(f"\n{method}:")
        print(f"  SSE Mean (abs): {sse['sse_mean']:.6f} m")
        print(f"  SSE Std Dev: {sse['sse_std']:.6f} m")
        print(f"  SSE RMS: {sse['sse_rms']:.6f} m")
    
    # Create updated comparison table with SSE
    comparison_file = results_dir / 'comparison_table_20251125_223231.csv'
    
    # Try different encodings
    encodings = ['utf-8', 'latin-1', 'cp1252', 'iso-8859-1']
    df_comp = None
    for enc in encodings:
        try:
            df_comp = pd.read_csv(comparison_file, encoding=enc)
            break
        except:
            continue
    
    if df_comp is not None:
        # Add SSE columns
        df_comp['SSE Mean (m)'] = [sse_results.get(m, {}).get('sse_mean', np.nan) for m in df_comp['Method']]
        df_comp['SSE Std (m)'] = [sse_results.get(m, {}).get('sse_std', np.nan) for m in df_comp['Method']]
        df_comp['SSE RMS (m)'] = [sse_results.get(m, {}).get('sse_rms', np.nan) for m in df_comp['Method']]
        
        # Save updated comparison table
        output_file = results_dir / 'comparison_table_with_sse.csv'
        df_comp.to_csv(output_file, index=False)
        print(f"\n{'='*70}")
        print(f"Updated comparison table saved to: {output_file.name}")
        print(f"{'='*70}")
        print("\nUpdated Table:")
        print(df_comp.to_string(index=False))
    
    # Print summary
    print(f"\n{'='*70}")
    print("SSE SUMMARY")
    print(f"{'='*70}")
    print(f"\n{'Method':<25} {'SSE Mean (m)':<15} {'SSE Std (m)':<15} {'SSE RMS (m)':<15}")
    print("-"*70)
    for method in ['Step PID', 'One-Shot Trajectory', 'Trajectory Update']:
        if method in sse_results:
            sse = sse_results[method]
            print(f"{method:<25} {sse['sse_mean']:>12.6f}   {sse['sse_std']:>12.6f}   {sse['sse_rms']:>12.6f}")
    
    # Find best
    best_sse = min(sse_results.items(), key=lambda x: x[1]['sse_mean'])
    print(f"\nBest SSE (lowest mean): {best_sse[0]} ({best_sse[1]['sse_mean']:.6f} m)")

if __name__ == '__main__':
    main()

