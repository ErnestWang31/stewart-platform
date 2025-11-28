# Real Hardware Experiments Guide

## Overview

This guide explains how to run the controller comparison experiments on your actual Stewart platform hardware.

## Prerequisites

1. **Hardware Setup:**
   - Stewart platform with 3 servos connected to Arduino
   - Camera connected and calibrated
   - Arduino uploaded with `stewart_platform_arduino.ino`
   - Platform calibrated (run `calibration_2d.py` first)

2. **Configuration:**
   - `config_stewart.json` must be properly configured with:
     - Camera index
     - Arduino COM port
     - PID gains
     - Platform geometry

## Running Experiments

### Quick Start

```bash
python run_comparison_experiments_hardware.py
```

### What Happens

1. **Initialization:**
   - Connects to Arduino
   - Opens camera
   - Waits for you to position the ball

2. **Three Experiments:**
   - **Experiment 1:** Step PID (setpoint = 0cm)
   - **Experiment 2:** One-shot Trajectory (fixed trajectory)
   - **Experiment 3:** Trajectory Update (replans every 0.2s, setpoint = 0cm)

3. **Between Experiments:**
   - Platform resets to neutral
   - You can reposition the ball
   - Press Enter to continue

4. **Data Collection:**
   - All data logged to CSV files in `results/` directory
   - Files named: `experiment*_hardware_*.csv`
   - Comparison table saved as `comparison_table_hardware.md`

## Safety Features

- **Emergency Stop:** Press `Ctrl+C` to stop any experiment immediately
- **Auto Reset:** Platform returns to neutral after each experiment
- **Angle Limits:** Control signals clipped to ±15 degrees

## Generating Plots

After running experiments, generate comparison plots:

```bash
# For hardware data
python plot_comparison.py --hardware

# Or use -h flag
python plot_comparison.py -h
```

## Tips

1. **Position the Ball:** Start with the ball at ~10cm from center for best results
2. **Stable Surface:** Ensure platform is on a stable, level surface
3. **Good Lighting:** Ensure adequate lighting for ball detection
4. **Check Camera:** Verify ball is being detected before starting
5. **Monitor:** Watch the platform during experiments for safety

## Troubleshooting

### Arduino Connection Failed
- Check COM port in `config_stewart.json`
- Verify Arduino is connected and powered
- Check that no other program is using the serial port

### Camera Not Opening
- Check camera index in `config_stewart.json`
- Verify camera is connected and not in use by another program
- Try different camera indices (0, 1, 2, etc.)

### Ball Not Detected
- Run calibration: `python calibration_2d.py`
- Adjust HSV color bounds in config
- Check lighting conditions

### Platform Not Moving
- Verify Arduino is receiving commands (check serial monitor)
- Check servo connections
- Verify motor direction settings in config

## File Structure

After running experiments, you'll have:

```
results/
├── experiment1_step_pid_hardware_YYYYMMDD_HHMMSS.csv
├── experiment2_trajectory_oneshot_hardware_YYYYMMDD_HHMMSS.csv
├── experiment3_trajectory_update_hardware_YYYYMMDD_HHMMSS.csv
└── comparison_table_hardware.md
```

## Comparison with Simulation

To compare hardware vs simulation results:

1. Run simulation: `python run_comparison_experiments.py`
2. Run hardware: `python run_comparison_experiments_hardware.py`
3. Generate plots for both:
   ```bash
   python plot_comparison.py          # Simulation
   python plot_comparison.py --hardware  # Hardware
   ```

