# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | 0.048 | 0.036333 | 0.098595 | 11.357 | 0.0% | 108.693 | 22.128 | 40 |
| One-Shot Trajectory | 4.640 | 0.034483 | 0.053460 | 4.879 | 0.0% | 77.434 | 27.316 | 30 |
| Trajectory Update | 2.720 | 0.039960 | 0.139951 | 15.000 | 2.2% | 108.329 | 26.581 | 32 |

## Detailed Metrics

### Step PID
- Completion Time: 0.048s
- RMSE: 0.036333 m
- Max Deviation: 0.098595 m
- Max Control: 11.357°
- Saturation: 0.0%
- Max Jerk: 108.693 m/s³
- RMS Jerk: 22.128 m/s³
- Velocity Reversals: 40

### One-Shot Trajectory
- Completion Time: 4.640s
- RMSE: 0.034483 m
- Max Deviation: 0.053460 m
- Max Control: 4.879°
- Saturation: 0.0%
- Max Jerk: 77.434 m/s³
- RMS Jerk: 27.316 m/s³
- Velocity Reversals: 30

### Trajectory Update
- Completion Time: 2.720s
- RMSE: 0.039960 m
- Max Deviation: 0.139951 m
- Max Control: 15.000°
- Saturation: 2.2%
- Max Jerk: 108.329 m/s³
- RMS Jerk: 26.581 m/s³
- Velocity Reversals: 32
