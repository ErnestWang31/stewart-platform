# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | N/A | 0.037918 | 0.135735 | 7.920 | 0.0% | 102.232 | 17.177 | 93 |
| One-Shot Trajectory | N/A | 0.033451 | 0.064447 | 5.328 | 0.0% | 101.633 | 25.988 | 70 |
| Trajectory Update | N/A | 0.035147 | 0.136758 | 15.000 | 1.5% | 136.651 | 17.012 | 28 |

## Detailed Metrics

### Step PID
- Completion Time: N/A
- RMSE: 0.037918 m
- Max Deviation: 0.135735 m
- Max Control: 7.920°
- Saturation: 0.0%
- Max Jerk: 102.232 m/s³
- RMS Jerk: 17.177 m/s³
- Velocity Reversals: 93

### One-Shot Trajectory
- Completion Time: N/A
- RMSE: 0.033451 m
- Max Deviation: 0.064447 m
- Max Control: 5.328°
- Saturation: 0.0%
- Max Jerk: 101.633 m/s³
- RMS Jerk: 25.988 m/s³
- Velocity Reversals: 70

### Trajectory Update
- Completion Time: N/A
- RMSE: 0.035147 m
- Max Deviation: 0.136758 m
- Max Control: 15.000°
- Saturation: 1.5%
- Max Jerk: 136.651 m/s³
- RMS Jerk: 17.012 m/s³
- Velocity Reversals: 28
