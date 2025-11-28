# Experiment Runner for Controller Comparison
# Runs three experiments: Step PID, One-shot Trajectory, Trajectory Update

import numpy as np
import time
import json
import os

from ball_tracker import SimulatedBallTracker
from pid_controller_1d import PIDController1D
from trajectory_generator import generate_trajectory, sample_trajectory
from trajectory_updater import TrajectoryUpdater
from acceleration_based_tracker import AccelerationBasedTracker, TiltAngleMapping
from data_logger import DataLogger
from metrics import compute_all_metrics

# Experiment parameters
INITIAL_POSITION = 0.10  # 10 cm in meters
TARGET_POSITION = 0.00   # 0 cm in meters
EXPERIMENT_DURATION = 50.0  # seconds (increased for better observation)
CONTROL_LIMIT = 15.0  # degrees
DT = 0.01  # Control loop time step (100 Hz)

# PID gains (load from config if available, otherwise use defaults)
DEFAULT_KP = 0.55
DEFAULT_KI = 0.1
DEFAULT_KD = 0.3

# Acceleration-based tracker gains (for trajectory tracking)
DEFAULT_ACCEL_KP = 10.0  # Position error gain
DEFAULT_ACCEL_KD = 5.0    # Velocity error gain

# Trajectory parameters
TRAJECTORY_TIME_HORIZON = 4.0  # seconds
TRAJECTORY_UPDATE_PERIOD = 0.2  # seconds

# Metrics parameters
COMPLETION_TOLERANCE = 0.005  # 5 mm
SETTLE_DURATION = 0.5  # seconds


def load_pid_gains_from_config(config_file="config_stewart.json"):
    """Load PID gains from config file.
    
    Args:
        config_file (str): Path to config file
        
    Returns:
        tuple: (Kp, Ki, Kd)
    """
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
            pid_config = config.get('pid', {})
            Kp = pid_config.get('Kp_x', DEFAULT_KP)
            Ki = pid_config.get('Ki_x', DEFAULT_KI)
            Kd = pid_config.get('Kd_x', DEFAULT_KD)
            return Kp, Ki, Kd
    except Exception as e:
        print(f"[CONFIG] Could not load config: {e}, using defaults")
        return DEFAULT_KP, DEFAULT_KI, DEFAULT_KD


def run_step_pid_experiment():
    """Run Experiment 1: Step PID (setpoint jumps from 10cm → 0cm at t=0).
    
    Returns:
        DataLogger: Logger with experiment data
    """
    print("\n" + "="*60)
    print("EXPERIMENT 1: Step PID")
    print("="*60)
    
    # Load PID gains
    Kp, Ki, Kd = load_pid_gains_from_config()
    print(f"PID Gains: Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
    
    # Initialize components
    ball_tracker = SimulatedBallTracker(initial_position=INITIAL_POSITION)
    pid = PIDController1D(Kp=Kp, Ki=Ki, Kd=Kd, output_limit=CONTROL_LIMIT)
    logger = DataLogger(experiment_name="experiment1_step_pid")
    
    # Set setpoint to target immediately
    pid.set_setpoint(TARGET_POSITION)
    
    # Run experiment
    start_time = time.time()
    current_time = 0.0
    
    print(f"Starting experiment: {INITIAL_POSITION*100:.1f}cm → {TARGET_POSITION*100:.1f}cm")
    print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
    
    while current_time < EXPERIMENT_DURATION:
        # Get current ball position
        position = ball_tracker.get_position()
        
        # Update PID controller
        control_signal = pid.update(position, dt=DT)
        
        # Check saturation
        saturated = abs(control_signal) >= CONTROL_LIMIT
        
        # Update ball dynamics with platform tilt
        tilt_angle = control_signal  # Control signal is tilt angle
        ball_tracker.update(tilt_angle, dt=DT)
        
        # Calculate error
        error = TARGET_POSITION - position
        
        # Log data
        logger.log(
            time=current_time,
            position_x=position,
            setpoint=TARGET_POSITION,
            error=error,
            control_signal=control_signal,
            tilt_angle=tilt_angle,
            saturated=saturated
        )
        
        # Update time
        current_time = time.time() - start_time
        
        # Small sleep to maintain timing
        time.sleep(max(0, DT - (time.time() - start_time - current_time)))
    
    logger.save()
    print(f"Experiment complete. Logged {len(logger.data)} data points.")
    return logger


def run_oneshot_trajectory_experiment():
    """Run Experiment 2: One-shot Trajectory (fixed trajectory from 10cm → 0cm).
    Uses acceleration-based tracking instead of velocity-based PID.
    
    Returns:
        DataLogger: Logger with experiment data
    """
    print("\n" + "="*60)
    print("EXPERIMENT 2: One-shot Trajectory (Acceleration-Based)")
    print("="*60)
    
    # Initialize acceleration-based tracker
    # Use gains that provide similar performance to PID
    accel_Kp = DEFAULT_ACCEL_KP
    accel_Kd = DEFAULT_ACCEL_KD
    print(f"Acceleration Tracker Gains: Kp={accel_Kp:.2f}, Kd={accel_Kd:.2f}")
    
    # Initialize components
    ball_tracker = SimulatedBallTracker(initial_position=INITIAL_POSITION)
    actuator_mapping = TiltAngleMapping(g=9.81, use_small_angle_approx=False)
    tracker = AccelerationBasedTracker(
        Kp=accel_Kp,
        Kd=accel_Kd,
        acceleration_limit=9.81,  # Limit to gravity
        actuator_mapping=actuator_mapping
    )
    logger = DataLogger(experiment_name="experiment2_trajectory_oneshot")
    
    # Generate fixed trajectory
    trajectory = generate_trajectory(
        INITIAL_POSITION,
        TARGET_POSITION,
        TRAJECTORY_TIME_HORIZON,
        method='polynomial'
    )
    print(f"Generated {TRAJECTORY_TIME_HORIZON:.1f}s polynomial trajectory")
    
    # Run experiment
    start_time = time.time()
    current_time = 0.0
    
    print(f"Starting experiment: {INITIAL_POSITION*100:.1f}cm → {TARGET_POSITION*100:.1f}cm")
    print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
    
    while current_time < EXPERIMENT_DURATION:
        # Get current ball state
        position = ball_tracker.get_position()
        velocity = ball_tracker.get_velocity()
        
        # Update acceleration-based tracker
        result = tracker.update(trajectory, current_time, position, velocity)
        
        # Extract control signal and errors
        control_signal = result['actuator_command']
        error = result['position_error']
        setpoint = result['desired_position']
        
        # Check saturation (limit to CONTROL_LIMIT degrees)
        control_signal = np.clip(control_signal, -CONTROL_LIMIT, CONTROL_LIMIT)
        saturated = abs(control_signal) >= CONTROL_LIMIT
        
        # Update ball dynamics with platform tilt
        tilt_angle = control_signal
        ball_tracker.update(tilt_angle, dt=DT)
        
        # Log data
        logger.log(
            time=current_time,
            position_x=position,
            setpoint=setpoint,
            error=error,
            control_signal=control_signal,
            tilt_angle=tilt_angle,
            saturated=saturated
        )
        
        # Update time
        current_time = time.time() - start_time
        
        # Small sleep to maintain timing
        time.sleep(max(0, DT - (time.time() - start_time - current_time)))
    
    logger.save()
    print(f"Experiment complete. Logged {len(logger.data)} data points.")
    return logger


def run_trajectory_update_experiment():
    """Run Experiment 3: Trajectory Update (replan every 0.2s using current position).
    Uses acceleration-based tracking instead of velocity-based PID.
    
    Returns:
        DataLogger: Logger with experiment data
    """
    print("\n" + "="*60)
    print("EXPERIMENT 3: Trajectory Update (Acceleration-Based)")
    print("="*60)
    
    # Initialize acceleration-based tracker
    accel_Kp = DEFAULT_ACCEL_KP
    accel_Kd = DEFAULT_ACCEL_KD
    print(f"Acceleration Tracker Gains: Kp={accel_Kp:.2f}, Kd={accel_Kd:.2f}")
    print(f"Update period: {TRAJECTORY_UPDATE_PERIOD:.1f}s")
    
    # Initialize components
    ball_tracker = SimulatedBallTracker(initial_position=INITIAL_POSITION)
    actuator_mapping = TiltAngleMapping(g=9.81, use_small_angle_approx=False)
    tracker = AccelerationBasedTracker(
        Kp=accel_Kp,
        Kd=accel_Kd,
        acceleration_limit=9.81,  # Limit to gravity
        actuator_mapping=actuator_mapping
    )
    trajectory_updater = TrajectoryUpdater(
        target_position=TARGET_POSITION,
        trajectory_duration=TRAJECTORY_TIME_HORIZON,
        update_interval=TRAJECTORY_UPDATE_PERIOD,
        method='polynomial'
    )
    logger = DataLogger(experiment_name="experiment3_trajectory_update")
    
    # Run experiment
    start_time = time.time()
    current_time = 0.0
    
    print(f"Starting experiment: {INITIAL_POSITION*100:.1f}cm → {TARGET_POSITION*100:.1f}cm")
    print(f"Duration: {EXPERIMENT_DURATION:.1f}s")
    
    while current_time < EXPERIMENT_DURATION:
        # Get current ball state
        position = ball_tracker.get_position()
        velocity = ball_tracker.get_velocity()
        
        # Update trajectory if needed
        trajectory_updater.update(position, current_time)
        
        # Get current trajectory from updater
        current_trajectory = trajectory_updater.current_trajectory
        if current_trajectory is None:
            # Fallback: create a simple trajectory to target
            current_trajectory = generate_trajectory(
                position,
                TARGET_POSITION,
                TRAJECTORY_TIME_HORIZON,
                method='polynomial'
            )
        
        # Compute time relative to current trajectory start
        if trajectory_updater.trajectory_start_time is not None:
            trajectory_time = current_time - trajectory_updater.trajectory_start_time
        else:
            trajectory_time = 0.0
        trajectory_time = max(0.0, trajectory_time)
        
        # Update acceleration-based tracker
        result = tracker.update(current_trajectory, trajectory_time, position, velocity)
        
        # Extract control signal and errors
        control_signal = result['actuator_command']
        error = result['position_error']
        setpoint = result['desired_position']
        
        # Check saturation (limit to CONTROL_LIMIT degrees)
        control_signal = np.clip(control_signal, -CONTROL_LIMIT, CONTROL_LIMIT)
        saturated = abs(control_signal) >= CONTROL_LIMIT
        
        # Update ball dynamics with platform tilt
        tilt_angle = control_signal
        ball_tracker.update(tilt_angle, dt=DT)
        
        # Log data
        logger.log(
            time=current_time,
            position_x=position,
            setpoint=setpoint,
            error=error,
            control_signal=control_signal,
            tilt_angle=tilt_angle,
            saturated=saturated
        )
        
        # Update time
        current_time = time.time() - start_time
        
        # Small sleep to maintain timing
        time.sleep(max(0, DT - (time.time() - start_time - current_time)))
    
    logger.save()
    print(f"Experiment complete. Logged {len(logger.data)} data points.")
    return logger


def generate_comparison_table(logger1, logger2, logger3):
    """Generate comparison table from three experiment loggers.
    
    Args:
        logger1 (DataLogger): Step PID experiment data
        logger2 (DataLogger): One-shot Trajectory experiment data
        logger3 (DataLogger): Trajectory Update experiment data
    """
    # Get data arrays
    data1 = logger1.get_data_arrays()
    data2 = logger2.get_data_arrays()
    data3 = logger3.get_data_arrays()
    
    # Compute metrics for each experiment
    metrics1 = compute_all_metrics(
        data1['position_x'],
        data1['time'],
        data1['error'],
        data1['control_signal'],
        tolerance=COMPLETION_TOLERANCE,
        settle_duration=SETTLE_DURATION,
        u_limit=CONTROL_LIMIT
    )
    
    metrics2 = compute_all_metrics(
        data2['position_x'],
        data2['time'],
        data2['error'],
        data2['control_signal'],
        tolerance=COMPLETION_TOLERANCE,
        settle_duration=SETTLE_DURATION,
        u_limit=CONTROL_LIMIT
    )
    
    metrics3 = compute_all_metrics(
        data3['position_x'],
        data3['time'],
        data3['error'],
        data3['control_signal'],
        tolerance=COMPLETION_TOLERANCE,
        settle_duration=SETTLE_DURATION,
        u_limit=CONTROL_LIMIT
    )
    
    # Format completion time
    def fmt_time(t):
        if t is None:
            return "N/A"
        return f"{t:.2f}s"
    
    def fmt_float(f):
        return f"{f:.4f}"
    
    def fmt_percent(p):
        return f"{p:.1f}%"
    
    # Generate markdown table
    table = f"""# Controller Comparison Results

## Experiment Parameters
- Initial Position: {INITIAL_POSITION*100:.1f} cm
- Target Position: {TARGET_POSITION*100:.1f} cm
- Experiment Duration: {EXPERIMENT_DURATION:.1f} s
- Control Limit: ±{CONTROL_LIMIT:.1f}°
- Completion Tolerance: {COMPLETION_TOLERANCE*1000:.1f} mm
- Settle Duration: {SETTLE_DURATION:.1f} s

## Comparison Table

| Method | Completion Time | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|----------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | {fmt_time(metrics1['completion_time'])} | {fmt_float(metrics1['rmse'])} | {fmt_float(metrics1['max_deviation'])} | {fmt_float(metrics1['max_control'])} | {fmt_percent(metrics1['saturation_percent'])} | {fmt_float(metrics1['max_jerk'])} | {fmt_float(metrics1['rms_jerk'])} | {metrics1['velocity_reversals']} |
| Traj 1-shot | {fmt_time(metrics2['completion_time'])} | {fmt_float(metrics2['rmse'])} | {fmt_float(metrics2['max_deviation'])} | {fmt_float(metrics2['max_control'])} | {fmt_percent(metrics2['saturation_percent'])} | {fmt_float(metrics2['max_jerk'])} | {fmt_float(metrics2['rms_jerk'])} | {metrics2['velocity_reversals']} |
| Traj Update | {fmt_time(metrics3['completion_time'])} | {fmt_float(metrics3['rmse'])} | {fmt_float(metrics3['max_deviation'])} | {fmt_float(metrics3['max_control'])} | {fmt_percent(metrics3['saturation_percent'])} | {fmt_float(metrics3['max_jerk'])} | {fmt_float(metrics3['rms_jerk'])} | {metrics3['velocity_reversals']} |

## Detailed Metrics

### Step PID
- Completion Time: {fmt_time(metrics1['completion_time'])}
- RMSE: {fmt_float(metrics1['rmse'])} m
- Max Deviation: {fmt_float(metrics1['max_deviation'])} m
- Max Control: {fmt_float(metrics1['max_control'])}°
- Saturation: {fmt_percent(metrics1['saturation_percent'])}
- Max Jerk: {fmt_float(metrics1['max_jerk'])} m/s³
- RMS Jerk: {fmt_float(metrics1['rms_jerk'])} m/s³
- Velocity Reversals: {metrics1['velocity_reversals']}

### One-shot Trajectory
- Completion Time: {fmt_time(metrics2['completion_time'])}
- RMSE: {fmt_float(metrics2['rmse'])} m
- Max Deviation: {fmt_float(metrics2['max_deviation'])} m
- Max Control: {fmt_float(metrics2['max_control'])}°
- Saturation: {fmt_percent(metrics2['saturation_percent'])}
- Max Jerk: {fmt_float(metrics2['max_jerk'])} m/s³
- RMS Jerk: {fmt_float(metrics2['rms_jerk'])} m/s³
- Velocity Reversals: {metrics2['velocity_reversals']}

### Trajectory Update
- Completion Time: {fmt_time(metrics3['completion_time'])}
- RMSE: {fmt_float(metrics3['rmse'])} m
- Max Deviation: {fmt_float(metrics3['max_deviation'])} m
- Max Control: {fmt_float(metrics3['max_control'])}°
- Saturation: {fmt_percent(metrics3['saturation_percent'])}
- Max Jerk: {fmt_float(metrics3['max_jerk'])} m/s³
- RMS Jerk: {fmt_float(metrics3['rms_jerk'])} m/s³
- Velocity Reversals: {metrics3['velocity_reversals']}
"""
    
    # Save table
    results_dir = "results"
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    table_file = os.path.join(results_dir, "comparison_table.md")
    with open(table_file, 'w') as f:
        f.write(table)
    
    print(f"\n[COMPARISON] Comparison table saved to {table_file}")
    print("\n" + "="*60)
    print("COMPARISON TABLE")
    print("="*60)
    print(table)
    
    return metrics1, metrics2, metrics3


def main():
    """Run all three experiments and generate comparison."""
    print("\n" + "="*60)
    print("CONTROLLER COMPARISON EXPERIMENTS")
    print("="*60)
    print(f"Initial Position: {INITIAL_POSITION*100:.1f} cm")
    print(f"Target Position: {TARGET_POSITION*100:.1f} cm")
    print(f"Experiment Duration: {EXPERIMENT_DURATION:.1f} s")
    print(f"Control Loop Rate: {1/DT:.0f} Hz")
    
    # Run experiments
    logger1 = run_step_pid_experiment()
    logger2 = run_oneshot_trajectory_experiment()
    logger3 = run_trajectory_update_experiment()
    
    # Generate comparison table
    metrics1, metrics2, metrics3 = generate_comparison_table(logger1, logger2, logger3)
    
    print("\n[SUCCESS] All experiments completed!")
    print("Run plot_comparison.py to generate visualization plots.")


if __name__ == "__main__":
    main()

