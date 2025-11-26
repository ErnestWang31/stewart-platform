# Quick Start Guide

## Running the Simulation Experiments

### Step 1: Run the Three Experiments

Run each experiment script to generate CSV data files. You can run them in any order:

```bash
# From the project root directory:

# Step PID experiment
python sim_suite/sim_step_pid.py

# One-Shot Trajectory experiment
python sim_suite/sim_oneshot_trajectory.py

# Trajectory Update experiment
python sim_suite/sim_trajectory_update.py
```

Or from within the `sim_suite/` directory:

```bash
cd sim_suite
python sim_step_pid.py
python sim_oneshot_trajectory.py
python sim_trajectory_update.py
```

Each experiment will:
- Print progress to the console
- Save a CSV file to `results/experiment_<type>_<timestamp>.csv`
- Display metrics at the end

### Step 2: Generate Comparison Plots

After running all three experiments, generate comparison plots and metrics:

```bash
# From project root:
python sim_suite/sim_compare_experiments.py

# Or from sim_suite directory:
cd sim_suite
python sim_compare_experiments.py
```

The comparison tool will:
- Automatically find the latest experiment files from each type
- Compute metrics for all three methods
- Display interactive plots (4 subplots showing position, error, control effort, and jerk)
- Save plots to `results/comparison_plots_<timestamp>.png`
- Save comparison table to `results/comparison_table_<timestamp>.md` and `.csv`
- Print metrics to console

### Optional: Analyze Individual Trajectory Update Results

To analyze a specific trajectory update experiment:

```bash
# Auto-detect latest file:
python sim_suite/sim_analyze_trajectory.py

# Or specify a file:
python sim_suite/sim_analyze_trajectory.py results/experiment_trajectory_update_YYYYMMDD_HHMMSS.csv
```

## Example Workflow

```bash
# 1. Run all three experiments
python sim_suite/sim_step_pid.py
python sim_suite/sim_oneshot_trajectory.py
python sim_suite/sim_trajectory_update.py

# 2. Compare results (auto-detects latest files)
python sim_suite/sim_compare_experiments.py

# 3. View results
# - Plots saved to: results/comparison_plots_<timestamp>.png
# - Table saved to: results/comparison_table_<timestamp>.md
# - Interactive plots displayed in matplotlib window
```

## Specifying Specific Files

If you want to compare specific experiment files (not just the latest):

```bash
python sim_suite/sim_compare_experiments.py \
    --step-pid results/experiment_step_pid_20251126_120000.csv \
    --oneshot results/experiment_oneshot_trajectory_20251126_120100.csv \
    --update results/experiment_trajectory_update_20251126_120200.csv
```

## Output Files

All results are saved to the `results/` directory:

- **Experiment CSVs**: `experiment_<type>_<timestamp>.csv`
- **Trajectory segments** (update only): `experiment_trajectory_update_<timestamp>_segments.json`
- **Comparison plots**: `comparison_plots_<timestamp>.png`
- **Comparison table**: `comparison_table_<timestamp>.md` and `.csv`

## Prerequisites

### Install Dependencies

```bash
# Install required packages
pip install numpy matplotlib

# Or from requirements file
pip install -r sim_suite/requirements.txt
```

**Required packages:**
- `numpy` - For numerical computations
- `matplotlib` - For plotting (only needed for comparison tool)

### Required Files

- `config_stewart.json` must exist in the project root directory
- The `results/` directory will be created automatically if it doesn't exist

## Notes

- All experiments use the same PID gains from `config_stewart.json`
- Simulation parameters can be adjusted in `sim_suite/sim_config.json`
- The comparison tool requires matplotlib for plotting
