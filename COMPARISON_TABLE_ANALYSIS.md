# Comparison Table Analysis

Analysis of experiment data from `comparison_table_20251125_223231.csv` using the same metrics framework as TEST_PLAN.md (lines 23-28).

## Summary Table

| Metric | Step PID | One-Shot Trajectory | Trajectory Update |
|--------|----------|---------------------|-------------------|
| **Completion Time (5mm)** | 3.233 s | 8.736 s | 4.544 s |
| **RMSE** | 0.0299 m | 0.0215 m | 0.0237 m |
| **Max Deviation** | 0.1367 m | 0.0692 m | 0.0912 m |
| **Max Control Signal** | 7.985 | 6.033 | 8.977 |
| **Saturation %** | 0.00% | 0.00% | 0.00% |
| **Max Jerk** | 101.6 m/s³ | 116.3 m/s³ | 95.8 m/s³ |
| **RMS Jerk** | 17.7 m/s³ | 19.1 m/s³ | 13.5 m/s³ |
| **Velocity Reversals** | 93 | 79 | 94 |
| **SSE Mean** | 0.0012 m | 0.0083 m | 0.0035 m |
| **SSE Std** | 0.0005 m | 0.0013 m | 0.0042 m |
| **SSE RMS** | 0.0013 m | 0.0084 m | 0.0044 m |

## Detailed Analysis

### 1. Completion Time (Time to reach and stay within 5mm tolerance)

- **Step PID**: 3.233 s ⭐ **Best (fastest)**
- **Trajectory Update**: 4.544 s
- **One-Shot Trajectory**: 8.736 s

**Analysis**: Step PID achieves the target fastest, followed by Trajectory Update. One-Shot Trajectory takes the longest, likely because it follows a smooth pre-planned trajectory path rather than taking the most direct route.

### 2. RMSE (Root Mean Square Error)

- **One-Shot Trajectory**: 0.0215 m ⭐ **Best (lowest)**
- **Trajectory Update**: 0.0237 m
- **Step PID**: 0.0299 m

**Analysis**: One-Shot Trajectory has the lowest overall error, followed closely by Trajectory Update. Step PID has the highest RMSE, which is consistent with its faster but more aggressive response.

### 3. Max Overshoot/Deviation

- **One-Shot Trajectory**: 0.0692 m ⭐ **Best (smallest)**
- **Trajectory Update**: 0.0912 m
- **Step PID**: 0.1367 m

**Analysis**: One-Shot Trajectory has the smallest maximum deviation, demonstrating better control over the response. Step PID has the largest deviation, which is expected for a step response without feedforward control.

### 4. Control Effort

**Max Control Signal:**
- **One-Shot Trajectory**: 6.033 ⭐ **Best (lowest)**
- **Step PID**: 7.985
- **Trajectory Update**: 8.977

**Saturation Percentage:**
- All experiments: 0.00% (no saturation occurred)

**Analysis**: One-Shot Trajectory requires the least control effort, while Trajectory Update requires the most. None of the experiments saturated the control signal, indicating good controller design.

### 5. Smoothness (Jerk and Velocity Reversals)

**Max Jerk:**
- **Trajectory Update**: 95.8 m/s³ ⭐ **Best (lowest)**
- **Step PID**: 101.6 m/s³
- **One-Shot Trajectory**: 116.3 m/s³

**RMS Jerk:**
- **Trajectory Update**: 13.5 m/s³ ⭐ **Best (lowest)**
- **Step PID**: 17.7 m/s³
- **One-Shot Trajectory**: 19.1 m/s³

**Velocity Reversals:**
- **One-Shot Trajectory**: 79 ⭐ **Best (fewest)**
- **Step PID**: 93
- **Trajectory Update**: 94

**Analysis**: Trajectory Update has the smoothest motion overall (lowest jerk values), while One-Shot Trajectory has the fewest velocity reversals. The trade-off shows that smoothness can be achieved through different means.

### 6. Steady-State Error (SSE)

**SSE Mean (absolute):**
- **Step PID**: 0.0012 m ⭐ **Best (lowest)**
- **Trajectory Update**: 0.0035 m
- **One-Shot Trajectory**: 0.0083 m

**SSE Standard Deviation:**
- **Step PID**: 0.0005 m ⭐ **Best (most consistent)**
- **One-Shot Trajectory**: 0.0013 m
- **Trajectory Update**: 0.0042 m

**SSE RMS:**
- **Step PID**: 0.0013 m ⭐ **Best (lowest)**
- **Trajectory Update**: 0.0044 m
- **One-Shot Trajectory**: 0.0084 m

**Analysis**: Step PID has the best steady-state accuracy with the lowest mean error (0.0012m) and most consistent performance (0.0005m std). This is interesting because Step PID had the highest RMSE overall, but once it reaches steady-state, it maintains the target position very accurately. One-Shot Trajectory has the highest steady-state error, which is surprising given its low overall RMSE.

## Overall Comparison

### Strengths by Method:

**Step PID:**
- ✅ Fastest completion time (3.23s)
- ✅ Best steady-state error (0.0012m mean)
- ✅ Most consistent steady-state (0.0005m std)
- ✅ Moderate control effort
- ✅ Good jerk performance (second best)
- ❌ Largest overshoot/deviation (0.137m)
- ❌ Highest RMSE (0.030m)

**One-Shot Trajectory:**
- ✅ Lowest RMSE (0.022m)
- ✅ Smallest max deviation (0.069m)
- ✅ Lowest control effort (6.03)
- ✅ Fewest velocity reversals (79)
- ❌ Slowest completion time (8.74s)
- ❌ Highest max jerk (116.3 m/s³)
- ❌ Highest steady-state error (0.0083m mean)

**Trajectory Update:**
- ✅ Best smoothness (lowest jerk: 95.8 m/s³ max, 13.5 m/s³ RMS)
- ✅ Good RMSE (0.024m)
- ✅ Moderate completion time (4.54s)
- ❌ Highest control effort (8.98)
- ❌ Most velocity reversals (94)

## Comparison with Previous Analysis

Comparing with the previous experiment run (from `EXPERIMENT_METRICS_ANALYSIS.md`):

| Metric | Previous Run | This Run | Change |
|--------|--------------|----------|--------|
| **Step PID Completion** | 1.377 s | 3.233 s | +1.856 s (slower) |
| **One-Shot Completion** | 4.080 s | 8.736 s | +4.656 s (slower) |
| **Trajectory Update Completion** | 3.040 s | 4.544 s | +1.504 s (slower) |
| **Step PID RMSE** | 0.0238 m | 0.0299 m | +0.0061 m (worse) |
| **One-Shot RMSE** | 0.0172 m | 0.0215 m | +0.0043 m (worse) |
| **Trajectory Update RMSE** | 0.0182 m | 0.0237 m | +0.0055 m (worse) |

**Observations:**
- All methods show slower completion times in this run
- All methods show slightly higher RMSE values
- This suggests different experimental conditions or controller tuning
- The relative performance ranking remains consistent across runs

## Recommendations

1. **For fastest response and best steady-state accuracy**: Use **Step PID** (3.23s, 0.0012m SSE, but expect 0.137m overshoot)
2. **For best overall accuracy and low overshoot**: Use **One-Shot Trajectory** (0.022m RMSE, 0.069m max deviation, but 8.74s and higher SSE)
3. **For smooth motion**: Use **Trajectory Update** (lowest jerk, 4.54s, moderate SSE, but highest control effort)

The choice depends on the application requirements:
- **Speed priority**: Step PID
- **Accuracy and precision**: One-Shot Trajectory
- **Smooth operation**: Trajectory Update

## Files Generated

- `results/comparison_table_analysis.png` - Visual comparison charts
- `results/comparison_table_summary.csv` - Summary metrics in CSV format

