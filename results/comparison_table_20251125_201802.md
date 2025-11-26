# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | N/A | 0.036444 | 0.137019 | 8.183 | 0.0% | 179.786 | 19.602 | 64 |
| One-Shot Trajectory | N/A | 0.053502 | 0.093362 | 12.803 | 0.0% | 228.007 | 53.029 | 28 |
| Trajectory Update | 4.142 | 0.034026 | 0.098025 | 10.710 | 0.0% | 207.333 | 28.790 | 72 |

## Detailed Metrics

### Step PID
- Completion Time: N/A
- RMSE: 0.036444 m
- Max Deviation: 0.137019 m
- Max Control: 8.183°
- Saturation: 0.0%
- Max Jerk: 179.786 m/s³
- RMS Jerk: 19.602 m/s³
- Velocity Reversals: 64

### One-Shot Trajectory
- Completion Time: N/A
- RMSE: 0.053502 m
- Max Deviation: 0.093362 m
- Max Control: 12.803°
- Saturation: 0.0%
- Max Jerk: 228.007 m/s³
- RMS Jerk: 53.029 m/s³
- Velocity Reversals: 28

### Trajectory Update
- Completion Time: 4.142s
- RMSE: 0.034026 m
- Max Deviation: 0.098025 m
- Max Control: 10.710°
- Saturation: 0.0%
- Max Jerk: 207.333 m/s³
- RMS Jerk: 28.790 m/s³
- Velocity Reversals: 72
