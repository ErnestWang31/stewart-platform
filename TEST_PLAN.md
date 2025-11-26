# Test Plan for Comparing Control Methods

## Overview
Compare three control methods:
1. **Step PID**: Instant setpoint change
2. **One-Shot Trajectory**: Fixed pre-calculated trajectory
3. **Trajectory Update**: Adaptive trajectory regeneration

---

## Test 1: Baseline Performance (No Disturbance)

### Purpose
Establish baseline performance metrics for each method

### Setup
- **Initial position**: 10cm from center
- **Target**: 0cm (center)
- **Duration**: 15 seconds
- **Disturbance**: None

### What to Measure
- Completion time (time to reach and stay within 5mm tolerance)
- RMSE (Root Mean Square Error)
- Max overshoot/deviation
- Control effort (max control signal, saturation %)
- Smoothness (jerk, velocity reversals)
- Steady-state error

### Expected Results
- **Step PID**: Fast but may overshoot
- **One-Shot**: Smooth but may stall initially
- **Trajectory Update**: Moderate performance, adaptive

---

## Test 2: Disturbance Rejection at Steady State

### Purpose
Test how each method handles disturbances when ball is at steady state

### Setup
- **Initial position**: 10cm from center
- **Target**: 0cm (center)
- **Duration**: 15 seconds
- **Disturbance**: Impulse at t=5.0s (after settling)
  - **Position disturbance**: 10cm for 1.0s
  - **OR Actuator disturbance**: 3° tilt for 1.0s

### What to Measure
- Recovery time (time to return to within tolerance after disturbance)
- Max deviation during disturbance
- Overshoot after disturbance
- Control effort during recovery

### Expected Results
- **Step PID**: Fast recovery but may overshoot
- **One-Shot**: Poor recovery (fixed trajectory doesn't adapt)
- **Trajectory Update**: Best recovery (adapts by regenerating trajectory)

---

## Test 3: Disturbance During Transient

### Purpose
Test how each method handles disturbances while moving to target

### Setup
- **Initial position**: 10cm from center
- **Target**: 0cm (center)
- **Duration**: 15 seconds
- **Disturbance**: Impulse at t=1.0s (during movement)
  - **Actuator disturbance**: 3° tilt for 0.5s

### What to Measure
- Effect on completion time
- Path deviation
- Recovery behavior

### Expected Results
- **Step PID**: May overshoot more
- **One-Shot**: May not reach target (trajectory doesn't adapt)
- **Trajectory Update**: Should adapt and still reach target

---

## Test 4: Continuous Disturbance

### Purpose
Test robustness to persistent disturbances

### Setup
- **Initial position**: 10cm from center
- **Target**: 0cm (center)
- **Duration**: 15 seconds
- **Disturbance**: Sinusoidal from t=5.0s to t=7.0s
  - **Amplitude**: 2cm oscillation
  - **Frequency**: 2 Hz

### What to Measure
- Steady-state error during disturbance
- Control effort
- Ability to maintain position

### Expected Results
- **Step PID**: May oscillate
- **One-Shot**: Poor (fixed trajectory)
- **Trajectory Update**: Should adapt continuously

---

## Test 5: Different Initial Conditions

### Purpose
Test performance with different starting positions

### Setup
- **Initial positions**: 5cm, 10cm, 15cm from center
- **Target**: 0cm (center)
- **Duration**: 15 seconds
- **Disturbance**: None

### What to Measure
- Consistency of performance across different initial conditions
- Scaling behavior

### Expected Results
- **Step PID**: Performance may vary with initial distance
- **One-Shot**: Should scale well (trajectory adapts to initial position)
- **Trajectory Update**: Should adapt well to any initial position

---

## Recommended Test Sequence

### For Quick Comparison:
1. **Test 1** (Baseline) - Run all 3 methods
2. **Test 2** (Steady-state disturbance) - Run all 3 methods with actuator disturbance

### For Comprehensive Analysis:
1. **Test 1** (Baseline)
2. **Test 2** (Steady-state disturbance) - Use actuator disturbance
3. **Test 3** (Transient disturbance) - Use actuator disturbance
4. **Test 4** (Continuous disturbance) - Optional

---

## Configuration for Each Test

### Step PID:
```python
# run_step_pid_experiment.py
self.disturbance = create_impulse_disturbance(time=5.0, magnitude=3.0, duration=1.0, apply_to='actuator')
self.disturbance_type = 'actuator'
```

### One-Shot Trajectory:
```python
# run_oneshot_trajectory_experiment.py
self.disturbance = create_impulse_disturbance(time=5.0, magnitude=3.0, duration=1.0, apply_to='actuator')
self.disturbance_type = 'actuator'
```

### Trajectory Update:
```python
# run_trajectory_update_experiment.py
self.disturbance = create_impulse_disturbance(time=5.0, magnitude=3.0, duration=1.0, apply_to='actuator')
self.disturbance_type = 'actuator'
```

---

## Metrics to Compare

After running tests, use `compare_experiments.py` to generate:
- Comparison table (completion time, RMSE, max deviation, etc.)
- Comparison plots (position, error, control, jerk)
- Visual comparison of all three methods

---

## Key Questions to Answer

1. **Which method is fastest?** (Completion time)
2. **Which method is smoothest?** (Jerk, velocity reversals)
3. **Which method handles disturbances best?** (Recovery time, max deviation)
4. **Which method uses least control effort?** (Max control, saturation %)
5. **Which method is most robust?** (Consistent performance across conditions)

---

## Notes

- Run each test **3 times** and average results for statistical significance
- Use **actuator disturbances** (not position) for more realistic and visible effects
- Apply disturbances at **t=5.0s** (after system has settled) for best visibility
- Compare using `compare_experiments.py` after collecting all data

