# Experiment Visualization Guide

## Quick Start

To visualize a single experiment in real-time:

```bash
python visualize_experiment.py [experiment_type]
```

### Experiment Types:
- `step_pid` - Step PID (setpoint jumps from 10cm → 0cm at t=0)
- `oneshot` - One-shot Trajectory (fixed polynomial trajectory)
- `update` - Trajectory Update (replans every 0.2s)

### Examples:

```bash
# Run Step PID experiment
python visualize_experiment.py step_pid

# Run One-shot Trajectory experiment
python visualize_experiment.py oneshot

# Run Trajectory Update experiment
python visualize_experiment.py update

# Interactive selection (no argument)
python visualize_experiment.py
```

## Visualization Features

The visualization shows 4 plots in real-time:

1. **Position Tracking** - Shows ball position x(t) and setpoint r(t) over time
2. **Tracking Error** - Shows error e(t) = r(t) - x(t)
3. **Control Signal** - Shows control output u(t) with saturation limits
4. **Platform View** - 2D view of ball position on the platform with live stats

## Controls

- Close the plot window to stop the experiment
- Press Ctrl+C in terminal to interrupt

## Running All Experiments

To run all three experiments and generate comparison data:

```bash
python run_comparison_experiments.py
```

Then generate comparison plots:

```bash
python plot_comparison.py
```

