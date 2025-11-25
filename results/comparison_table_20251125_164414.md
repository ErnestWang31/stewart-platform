# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | 1.543 | 0.040343 | 0.130921 | 15.000 | 0.8% | 104.160 | 18.764 | 11 |
| One-Shot Trajectory | N/A | 0.033986 | 0.054527 | 15.000 | 0.6% | 72.009 | 24.526 | 32 |
| Trajectory Update | 2.720 | 0.039960 | 0.139951 | 15.000 | 2.2% | 108.329 | 26.581 | 32 |

## Detailed Metrics

### Step PID
- Completion Time: 1.543s
- RMSE: 0.040343 m
- Max Deviation: 0.130921 m
- Max Control: 15.000°
- Saturation: 0.8%
- Max Jerk: 104.160 m/s³
- RMS Jerk: 18.764 m/s³
- Velocity Reversals: 11

### One-Shot Trajectory
- Completion Time: N/A
- RMSE: 0.033986 m
- Max Deviation: 0.054527 m
- Max Control: 15.000°
- Saturation: 0.6%
- Max Jerk: 72.009 m/s³
- RMS Jerk: 24.526 m/s³
- Velocity Reversals: 32

### Trajectory Update
- Completion Time: 2.720s
- RMSE: 0.039960 m
- Max Deviation: 0.139951 m
- Max Control: 15.000°
- Saturation: 2.2%
- Max Jerk: 108.329 m/s³
- RMS Jerk: 26.581 m/s³
- Velocity Reversals: 32
