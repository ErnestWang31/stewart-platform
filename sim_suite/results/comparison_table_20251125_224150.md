# Controller Comparison Results

## Comparison Table

| Method | Completion Time (s) | RMSE (m) | Max Deviation (m) | Max Control (°) | Saturation % | Max Jerk (m/s³) | RMS Jerk (m/s³) | Vel. Reversals |
|--------|---------------------|----------|-------------------|------------------|--------------|-----------------|-----------------|----------------|
| Step PID | 6.134 | 0.039486 | 0.142779 | 11.952 | 0.0% | 727.627 | 213.930 | 136 |
| One-Shot Trajectory | 8.934 | 0.008681 | 0.024153 | 9.046 | 0.0% | 715.162 | 249.529 | 185 |
| Trajectory Update | 7.834 | 0.014045 | 0.039200 | 9.329 | 0.0% | 771.477 | 239.281 | 164 |

## Detailed Metrics

### Step PID
- Completion Time: 6.134s
- RMSE: 0.039486 m
- Max Deviation: 0.142779 m
- Max Control: 11.952°
- Saturation: 0.0%
- Max Jerk: 727.627 m/s³
- RMS Jerk: 213.930 m/s³
- Velocity Reversals: 136

### One-Shot Trajectory
- Completion Time: 8.934s
- RMSE: 0.008681 m
- Max Deviation: 0.024153 m
- Max Control: 9.046°
- Saturation: 0.0%
- Max Jerk: 715.162 m/s³
- RMS Jerk: 249.529 m/s³
- Velocity Reversals: 185

### Trajectory Update
- Completion Time: 7.834s
- RMSE: 0.014045 m
- Max Deviation: 0.039200 m
- Max Control: 9.329°
- Saturation: 0.0%
- Max Jerk: 771.477 m/s³
- RMS Jerk: 239.281 m/s³
- Velocity Reversals: 164
