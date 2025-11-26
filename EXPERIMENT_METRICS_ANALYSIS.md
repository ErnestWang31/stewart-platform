# Experiment Metrics Analysis

Analysis of three experiments based on TEST_PLAN.md metrics (lines 23-28):
- `experiment_oneshot_trajectory_20251125_212903.csv`
- `experiment_trajectory_update_20251125_212946.csv`
- `experiment_step_pid_20251125_213037.csv`

## Summary Table

| Metric | Oneshot Trajectory | Trajectory Update | Step PID |
|--------|-------------------|-------------------|----------|
| **Completion Time (5mm)** | 4.080 s | 3.040 s | 1.377 s |
| **RMSE** | 0.0172 m | 0.0182 m | 0.0238 m |
| **Max Overshoot** | 0.0502 m | 0.0622 m | 0.1349 m |
| **Max Deviation** | 0.0525 m | 0.0803 m | 0.1349 m |
| **Max Control Signal** | 4.758 | 8.282 | 7.612 |
| **Saturation %** | 0.00% | 0.00% | 0.00% |
| **Max Jerk** | 160.5 m/s³ | 181.6 m/s³ | 81.4 m/s³ |
| **Mean Abs Jerk** | 9.93 m/s³ | 10.41 m/s³ | 7.23 m/s³ |
| **Velocity Reversals** | 227 | 292 | 233 |
| **Steady-State Error (mean)** | 0.0018 m | 0.0007 m | 0.0014 m |
| **Steady-State Error (std)** | 0.0021 m | 0.0003 m | 0.0008 m |

## Detailed Analysis

### 1. Completion Time (Time to reach and stay within 5mm tolerance of final setpoint)

- **Step PID**: 1.377 s ⭐ **Best (fastest)**
- **Trajectory Update**: 3.040 s
- **Oneshot Trajectory**: 4.080 s

**Analysis**: The step PID achieves the target fastest because it's a direct step response to the final setpoint (0.0m). The trajectory update method is moderate, while the oneshot trajectory takes longest because it follows a smooth pre-planned trajectory path rather than taking the most direct route.

### 2. RMSE (Root Mean Square Error)

- **Oneshot Trajectory**: 0.0172 m ⭐ **Best**
- **Trajectory Update**: 0.0182 m
- **Step PID**: 0.0238 m

**Analysis**: All three methods have similar RMSE values, with oneshot trajectory having the lowest overall error. The differences are relatively small (within 0.0066 m).

### 3. Max Overshoot/Deviation

- **Oneshot Trajectory**: 0.0502 m overshoot, 0.0525 m max deviation ⭐ **Best**
- **Trajectory Update**: 0.0622 m overshoot, 0.0803 m max deviation
- **Step PID**: 0.1349 m overshoot, 0.1349 m max deviation

**Analysis**: The oneshot trajectory has the smallest overshoot and deviation. The step PID has the largest overshoot, which is expected for a step response without feedforward control.

### 4. Control Effort

**Max Control Signal:**
- **Oneshot Trajectory**: 4.758 ⭐ **Best (lowest)**
- **Step PID**: 7.612
- **Trajectory Update**: 8.282

**Saturation Percentage:**
- All experiments: 0.00% (no saturation occurred)

**Analysis**: The oneshot trajectory requires the least control effort, while trajectory update requires the most. None of the experiments saturated the control signal, indicating good controller design.

### 5. Smoothness (Jerk and Velocity Reversals)

**Max Jerk:**
- **Step PID**: 81.4 m/s³ ⭐ **Best (lowest)**
- **Oneshot Trajectory**: 160.5 m/s³
- **Trajectory Update**: 181.6 m/s³

**Mean Absolute Jerk:**
- **Step PID**: 7.23 m/s³ ⭐ **Best (lowest)**
- **Oneshot Trajectory**: 9.93 m/s³
- **Trajectory Update**: 10.41 m/s³

**Velocity Reversals:**
- **Oneshot Trajectory**: 227 ⭐ **Best (lowest)**
- **Step PID**: 233
- **Trajectory Update**: 292

**Analysis**: The step PID has the smoothest motion (lowest jerk), but the oneshot trajectory has fewer velocity reversals. The trajectory update method has the highest jerk and most reversals, suggesting less smooth motion.

### 6. Steady-State Error

**Mean Steady-State Error:**
- **Trajectory Update**: 0.0007 m ⭐ **Best (lowest)**
- **Step PID**: 0.0014 m
- **Oneshot Trajectory**: 0.0018 m

**Steady-State Error Standard Deviation:**
- **Trajectory Update**: 0.0003 m ⭐ **Best (lowest, most consistent)**
- **Step PID**: 0.0008 m
- **Oneshot Trajectory**: 0.0021 m

**Analysis**: The trajectory update method has the best steady-state accuracy and consistency. The oneshot trajectory has higher steady-state error and more variability.

## Overall Comparison

### Strengths by Method:

**Oneshot Trajectory:**
- ✅ Lowest RMSE
- ✅ Smallest overshoot/deviation
- ✅ Lowest control effort
- ✅ Fewest velocity reversals
- ❌ Slowest completion time (follows smooth trajectory)
- ❌ Higher steady-state error
- ❌ Higher jerk (less smooth)

**Trajectory Update:**
- ✅ Best steady-state accuracy
- ✅ Most consistent steady-state error
- ✅ Good RMSE
- ❌ Slowest completion time
- ❌ Highest control effort
- ❌ Highest jerk and most velocity reversals

**Step PID:**
- ✅ Fastest completion time
- ✅ Smoothest motion (lowest jerk)
- ✅ Moderate control effort
- ❌ Largest overshoot
- ❌ Highest RMSE

## Recommendations

1. **For fastest response**: Use **Step PID** (1.38s to target)
2. **For best steady-state accuracy**: Use **Trajectory Update** (0.0007m mean error)
3. **For low overshoot and smooth path**: Use **Oneshot Trajectory** (but slower, 4.08s)

The choice depends on the application requirements:
- **Speed**: Step PID (fastest, but largest overshoot)
- **Final accuracy**: Trajectory Update (best steady-state, moderate speed)
- **Low overshoot and smooth path**: Oneshot Trajectory (slowest but most controlled)

## Files Generated

- `results/experiment_metrics_comparison.csv` - Detailed metrics in CSV format
- `results/experiment_metrics_comparison.png` - Visual comparison charts

