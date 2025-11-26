# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | N/A | 0.036444 | 0.137019 | 8.183 | 0.0% | 179.786 | 19.602 | 64 |
| One-Shot Trajectory | N/A | 0.029457 | 0.064615 | 8.216 | 0.0% | 116.671 | 22.804 | 77 |
| Trajectory Update | N/A | 0.031113 | 0.084841 | 9.050 | 0.0% | 147.402 | 26.262 | 85 |

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
- RMSE: 0.029457 m
- Max Deviation: 0.064615 m
- Max Control: 8.216°
- Saturation: 0.0%
- Max Jerk: 116.671 m/s³
- RMS Jerk: 22.804 m/s³
- Velocity Reversals: 77

### Trajectory Update
- Completion Time: N/A
- RMSE: 0.031113 m
- Max Deviation: 0.084841 m
- Max Control: 9.050°
- Saturation: 0.0%
- Max Jerk: 147.402 m/s³
- RMS Jerk: 26.262 m/s³
- Velocity Reversals: 85
