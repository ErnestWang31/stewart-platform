# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | N/A | 0.034372 | 0.140899 | 10.258 | 0.0% | 606.782 | 221.473 | 176 |
| One-Shot Trajectory | 6.934 | 0.006400 | 0.020471 | 7.861 | 0.0% | 718.449 | 241.516 | 181 |
| Trajectory Update | 6.202 | 0.009539 | 0.030318 | 9.495 | 0.0% | 753.019 | 234.030 | 174 |

## Detailed Metrics

### Step PID
- Completion Time: N/A
- RMSE: 0.034372 m
- Max Deviation: 0.140899 m
- Max Control: 10.258°
- Saturation: 0.0%
- Max Jerk: 606.782 m/s³
- RMS Jerk: 221.473 m/s³
- Velocity Reversals: 176

### One-Shot Trajectory
- Completion Time: 6.934s
- RMSE: 0.006400 m
- Max Deviation: 0.020471 m
- Max Control: 7.861°
- Saturation: 0.0%
- Max Jerk: 718.449 m/s³
- RMS Jerk: 241.516 m/s³
- Velocity Reversals: 181

### Trajectory Update
- Completion Time: 6.202s
- RMSE: 0.009539 m
- Max Deviation: 0.030318 m
- Max Control: 9.495°
- Saturation: 0.0%
- Max Jerk: 753.019 m/s³
- RMS Jerk: 234.030 m/s³
- Velocity Reversals: 174
