# Side-by-Side Comparison: Both Experiment Runs

## Summary Tables

### Run 1: Individual Experiment Files (20251125_212903, 212946, 213037)

| Metric | Step PID | One-Shot Trajectory | Trajectory Update |
|--------|----------|---------------------|-------------------|
| **Completion Time** | 1.377 s | 4.080 s | 3.040 s |
| **RMSE** | 0.0238 m | 0.0172 m | 0.0182 m |
| **Max Deviation** | 0.1349 m | 0.0525 m | 0.0803 m |
| **Max Control Signal** | 7.612 | 4.758 | 8.282 |
| **Saturation %** | 0.00% | 0.00% | 0.00% |
| **Max Jerk** | 81.4 m/s³ | 160.5 m/s³ | 181.6 m/s³ |
| **Mean Abs Jerk** | 7.23 m/s³ | 9.93 m/s³ | 10.41 m/s³ |
| **Velocity Reversals** | 233 | 227 | 292 |
| **Steady-State Error** | 0.0014 m | 0.0018 m | 0.0007 m |

### Run 2: Comparison Table (20251125_223231)

| Metric | Step PID | One-Shot Trajectory | Trajectory Update |
|--------|----------|---------------------|-------------------|
| **Completion Time** | 3.233 s | 8.736 s | 4.544 s |
| **RMSE** | 0.0299 m | 0.0215 m | 0.0237 m |
| **Max Deviation** | 0.1367 m | 0.0692 m | 0.0912 m |
| **Max Control Signal** | 7.985 | 6.033 | 8.977 |
| **Saturation %** | 0.00% | 0.00% | 0.00% |
| **Max Jerk** | 101.6 m/s³ | 116.3 m/s³ | 95.8 m/s³ |
| **RMS Jerk** | 17.7 m/s³ | 19.1 m/s³ | 13.5 m/s³ |
| **Velocity Reversals** | 93 | 79 | 94 |

## Key Differences Between Runs

### Completion Time
- **Run 1**: Step PID fastest (1.38s), One-Shot slowest (4.08s)
- **Run 2**: Step PID fastest (3.23s), One-Shot slowest (8.74s)
- **Change**: All methods slower in Run 2, especially One-Shot (+4.66s)

### RMSE
- **Run 1**: One-Shot best (0.0172m), Step PID worst (0.0238m)
- **Run 2**: One-Shot best (0.0215m), Step PID worst (0.0299m)
- **Change**: All methods show higher RMSE in Run 2

### Max Deviation
- **Run 1**: One-Shot best (0.0525m), Step PID worst (0.1349m)
- **Run 2**: One-Shot best (0.0692m), Step PID worst (0.1367m)
- **Change**: Similar relative performance, One-Shot slightly worse in Run 2

### Control Effort
- **Run 1**: One-Shot lowest (4.758), Trajectory Update highest (8.282)
- **Run 2**: One-Shot lowest (6.033), Trajectory Update highest (8.977)
- **Change**: All methods require more control effort in Run 2

### Smoothness (Jerk)
- **Run 1**: Step PID smoothest (81.4 m/s³), Trajectory Update roughest (181.6 m/s³)
- **Run 2**: Trajectory Update smoothest (95.8 m/s³), One-Shot roughest (116.3 m/s³)
- **Change**: Significant difference - Run 2 shows Trajectory Update as smoothest

### Velocity Reversals
- **Run 1**: One-Shot fewest (227), Trajectory Update most (292)
- **Run 2**: One-Shot fewest (79), Trajectory Update most (94)
- **Change**: Dramatic reduction in reversals for all methods in Run 2

## Consistent Patterns Across Both Runs

1. **Step PID**: Always fastest completion time, but largest overshoot
2. **One-Shot Trajectory**: Always lowest RMSE and smallest max deviation
3. **Trajectory Update**: Consistently moderate performance across most metrics
4. **No Saturation**: Neither run shows any control signal saturation
5. **Relative Rankings**: The relative performance order is consistent across runs

## Possible Explanations for Differences

1. **Different Experimental Conditions**: 
   - Different initial conditions
   - Different controller tuning parameters
   - Different trajectory planning parameters

2. **Different Measurement Methods**:
   - Run 1: Calculated from raw time-series data
   - Run 2: Pre-calculated metrics (possibly different algorithms)

3. **Different Trajectory Characteristics**:
   - Run 2 may use different trajectory profiles
   - Different update frequencies or strategies

## Recommendations

### For Run 1 Characteristics:
- **Speed**: Step PID (1.38s)
- **Accuracy**: One-Shot Trajectory (0.0172m RMSE)
- **Smoothness**: Step PID (81.4 m/s³ max jerk)

### For Run 2 Characteristics:
- **Speed**: Step PID (3.23s)
- **Accuracy**: One-Shot Trajectory (0.0215m RMSE)
- **Smoothness**: Trajectory Update (95.8 m/s³ max jerk, 13.5 m/s³ RMS)

### Overall Best Choice:
- **One-Shot Trajectory** consistently provides best accuracy and lowest overshoot
- **Step PID** consistently provides fastest response
- **Trajectory Update** provides good balance, with best smoothness in Run 2

