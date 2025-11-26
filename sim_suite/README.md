# Stewart Platform Simulation Suite

This directory contains a complete simulation suite that replicates the three experiment types (Step PID, One-Shot Trajectory, and Trajectory Update) using a physics-based simulator instead of real hardware.

## Overview

The simulation suite models:
- **Ball dynamics**: Mass, friction/damping, boundary constraints
- **Motor actuation**: First-order lag, rate limiting, angle saturation
- **Sensor characteristics**: Gaussian noise, transport delay, sampling rate

All simulation experiments produce data in the same CSV format as real experiments, making them compatible with existing analysis tools.

## Files

- `physics_engine.py` - Core physics simulation engine
- `sim_step_pid.py` - Simulated Step PID experiment
- `sim_oneshot_trajectory.py` - Simulated One-Shot Trajectory experiment
- `sim_trajectory_update.py` - Simulated Trajectory Update experiment
- `sim_analyze_trajectory.py` - Analysis script for trajectory data
- `sim_compare_experiments.py` - Comparison tool for all three methods
- `sim_config.json` - Simulation-specific configuration parameters

## Configuration

The simulation uses two configuration files:

1. **`config_stewart.json`** (parent directory) - Main platform configuration
   - PID gains
   - Platform geometry
   - Platform limits

2. **`sim_config.json`** - Simulation-specific parameters:
   - `ball_mass`: Ball mass in kg (default: 0.1)
   - `friction_coefficient`: Viscous damping coefficient in N·s/m (default: 0.2)
   - `motor_time_constant`: Motor first-order lag time constant in seconds (default: 0.1)
   - `motor_max_rate_deg_per_s`: Maximum motor angular velocity in deg/s (default: 180.0)
   - `sensor_noise_std`: Sensor noise standard deviation in meters (default: 0.002)
   - `sensor_delay_seconds`: Sensor transport delay in seconds (default: 0.1)
   - `sensor_rate_hz`: Sensor sampling rate in Hz (default: 30.0)
   - `physics_dt`: Physics integration time step in seconds (default: 0.002)
   - `initial_position_x`: Initial ball X position in meters (default: -0.10)
   - `initial_position_y`: Initial ball Y position in meters (default: 0.0)

## Usage

### Running Experiments

All simulation scripts can be run from the `sim_suite/` directory:

```bash
# Step PID experiment
python sim_step_pid.py

# One-Shot Trajectory experiment
python sim_oneshot_trajectory.py

# Trajectory Update experiment
python sim_trajectory_update.py
```

Or from the parent directory:

```bash
python sim_suite/sim_step_pid.py
python sim_suite/sim_oneshot_trajectory.py
python sim_suite/sim_trajectory_update.py
```

### Analyzing Results

```bash
# Analyze trajectory update data
python sim_suite/sim_analyze_trajectory.py [filename.csv]

# Compare all three experiment types
python sim_suite/sim_compare_experiments.py
```

The comparison tool will automatically find the latest experiment files, or you can specify them:

```bash
python sim_suite/sim_compare_experiments.py \
    --step-pid results/experiment_step_pid_YYYYMMDD_HHMMSS.csv \
    --oneshot results/experiment_oneshot_trajectory_YYYYMMDD_HHMMSS.csv \
    --update results/experiment_trajectory_update_YYYYMMDD_HHMMSS.csv
```

## Output Format

All experiments save data to the `results/` directory in the same format as real experiments:

- **CSV files**: `experiment_<type>_<timestamp>.csv`
  - Columns: `time, position_x, setpoint, error, control_signal, tilt_angle, saturated`
- **JSON files** (trajectory update only): `experiment_trajectory_update_<timestamp>_segments.json`
  - Contains trajectory segment information for plotting

## Physics Model

### Ball Dynamics

The ball is modeled as a point mass on a tilted plane with viscous friction:

```
a = g·sin(θ) - (c/m)·v
```

Where:
- `a`: Acceleration
- `g`: Gravity (9.81 m/s²)
- `θ`: Platform tilt angle
- `c`: Friction coefficient
- `m`: Ball mass
- `v`: Ball velocity

The ball is constrained to stay within the platform radius (0.15 m).

### Motor Actuation

Motors are modeled with first-order lag and rate limiting:

```
θ_new = θ_old + (θ_cmd - θ_old) · (1 - exp(-dt/τ))
```

With rate limiting:
```
|dθ/dt| ≤ motor_max_rate
```

And angle saturation:
```
|θ| ≤ 30°
```

Note: The actual platform can achieve ±30° tilt because with three motors, one can be at +15° while the other two are at -15°, giving a net tilt of 30° along that axis.

### Sensor Model

The sensor adds Gaussian noise and transport delay:

```
measured = true_position + N(0, σ²)
```

With a delay buffer that holds samples for `sensor_delay_seconds` before output.

## Tuning Parameters

To match real hardware behavior, adjust these parameters in `sim_config.json`:

- **Ball mass**: Affects acceleration and inertia
- **Friction coefficient**: Higher values = more damping, slower response
- **Motor time constant**: Lower values = faster motor response
- **Motor max rate**: Limits how fast motors can change angle
- **Sensor noise**: Adds measurement uncertainty
- **Sensor delay**: Simulates camera processing latency

## Dependencies

### Python Packages

Install required packages:

```bash
pip install numpy matplotlib
```

Or install from the requirements file:

```bash
pip install -r sim_suite/requirements.txt
```

**Required packages:**
- `numpy` (>=1.20.0) - For numerical computations and physics calculations
- `matplotlib` (>=3.3.0) - For plotting and visualization (only needed for comparison tool)

### Code Dependencies

The simulation suite reuses existing modules from the parent directory:
- `pid_controller_2d.py`
- `trajectory_generator.py`
- `trajectory_updater.py`
- `metrics.py`

Make sure these files are accessible (either in the parent directory or in your Python path).

## Notes

- The simulation runs at a fixed physics timestep (default 0.002s = 500 Hz)
- Control runs at the sensor rate (default 30 Hz)
- All experiments start with the ball at the initial position specified in config
- Results are saved to the same `results/` directory as real experiments
- The comparison tool works with both simulated and real experiment data
