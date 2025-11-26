# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | 3.233 | 0.029896 | 0.136720 | 7.985 | 0.0% | 101.576 | 17.681 | 93 |
| One-Shot Trajectory | 8.736 | 0.021535 | 0.069206 | 6.033 | 0.0% | 116.337 | 19.103 | 79 |
| Trajectory Update | 4.544 | 0.023658 | 0.091202 | 8.977 | 0.0% | 95.778 | 13.535 | 94 |

## Detailed Metrics

### Step PID
- Completion Time: 3.233s
- RMSE: 0.029896 m
- Max Deviation: 0.136720 m
- Max Control: 7.985°
- Saturation: 0.0%
- Max Jerk: 101.576 m/s³
- RMS Jerk: 17.681 m/s³
- Velocity Reversals: 93

### One-Shot Trajectory
- Completion Time: 8.736s
- RMSE: 0.021535 m
- Max Deviation: 0.069206 m
- Max Control: 6.033°
- Saturation: 0.0%
- Max Jerk: 116.337 m/s³
- RMS Jerk: 19.103 m/s³
- Velocity Reversals: 79

### Trajectory Update
- Completion Time: 4.544s
- RMSE: 0.023658 m
- Max Deviation: 0.091202 m
- Max Control: 8.977°
- Saturation: 0.0%
- Max Jerk: 95.778 m/s³
- RMS Jerk: 13.535 m/s³
- Velocity Reversals: 94
