# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | 5.696 | 0.023793 | 0.134871 | 7.612 | 0.0% | 81.430 | 12.176 | 233 |
| One-Shot Trajectory | 5.537 | 0.017185 | 0.052541 | 4.758 | 0.0% | 160.527 | 18.291 | 227 |
| Trajectory Update | 5.569 | 0.018186 | 0.080267 | 8.282 | 0.0% | 181.636 | 20.161 | 292 |

## Detailed Metrics

### Step PID
- Completion Time: 5.696s
- RMSE: 0.023793 m
- Max Deviation: 0.134871 m
- Max Control: 7.612°
- Saturation: 0.0%
- Max Jerk: 81.430 m/s³
- RMS Jerk: 12.176 m/s³
- Velocity Reversals: 233

### One-Shot Trajectory
- Completion Time: 5.537s
- RMSE: 0.017185 m
- Max Deviation: 0.052541 m
- Max Control: 4.758°
- Saturation: 0.0%
- Max Jerk: 160.527 m/s³
- RMS Jerk: 18.291 m/s³
- Velocity Reversals: 227

### Trajectory Update
- Completion Time: 5.569s
- RMSE: 0.018186 m
- Max Deviation: 0.080267 m
- Max Control: 8.282°
- Saturation: 0.0%
- Max Jerk: 181.636 m/s³
- RMS Jerk: 20.161 m/s³
- Velocity Reversals: 292
