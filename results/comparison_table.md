# Controller Comparison Results

## Experiment Parameters
- Initial Position: 10.0 cm
- Target Position: 0.0 cm
- Experiment Duration: 6.0 s
- Control Limit: ±15.0°
- Completion Tolerance: 5.0 mm
- Settle Duration: 0.5 s

## Comparison Table

| Method | Completion Time | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|----------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | N/A | 16.1626 | 35.1532 | 15.0000 | 96.9% | 94181749273239.7031 | 3910684280930.4082 | 0 |
| Traj 1-shot | N/A | 12.0986 | 28.0376 | 15.0000 | 84.9% | 8091.4155 | 2475.0175 | 1 |
| Traj Update | N/A | 0.1021 | 0.5652 | 15.0000 | 23.2% | 1919.7289 | 336.7611 | 1 |

## Detailed Metrics

### Step PID
- Completion Time: N/A
- RMSE: 16.1626 m
- Max Deviation: 35.1532 m
- Max Control: 15.0000°
- Saturation: 96.9%
- Max Jerk: 94181749273239.7031 m/s³
- RMS Jerk: 3910684280930.4082 m/s³
- Velocity Reversals: 0

### One-shot Trajectory
- Completion Time: N/A
- RMSE: 12.0986 m
- Max Deviation: 28.0376 m
- Max Control: 15.0000°
- Saturation: 84.9%
- Max Jerk: 8091.4155 m/s³
- RMS Jerk: 2475.0175 m/s³
- Velocity Reversals: 1

### Trajectory Update
- Completion Time: N/A
- RMSE: 0.1021 m
- Max Deviation: 0.5652 m
- Max Control: 15.0000°
- Saturation: 23.2%
- Max Jerk: 1919.7289 m/s³
- RMS Jerk: 336.7611 m/s³
- Velocity Reversals: 1
