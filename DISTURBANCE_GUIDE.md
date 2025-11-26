# How to Add Disturbances to Experiments

## Quick Start

### 1. Step Disturbance (Sudden Push)
Add a sudden position change at a specific time:

```python
# In __init__ method, add:
from disturbance import create_step_disturbance
self.disturbance = create_step_disturbance(time=2.0, magnitude=0.05)  # 5cm push at t=2s
```

### 2. Impulse Disturbance (Temporary Push)
Add a temporary position change:

```python
from disturbance import create_impulse_disturbance
self.disturbance = create_impulse_disturbance(time=2.0, magnitude=0.05, duration=0.5)
# 5cm push for 0.5 seconds starting at t=2s
```

### 3. Sinusoidal Disturbance (Oscillating Push)
Add an oscillating position change:

```python
from disturbance import create_sinusoidal_disturbance
self.disturbance = create_sinusoidal_disturbance(
    start_time=2.0, 
    amplitude=0.02,  # 2cm oscillation
    frequency=2.0,   # 2 Hz
    duration=1.0     # 1 second
)
```

## How It Works

The disturbance is applied to the **measured position** before it's used by the controller. This simulates:
- Pushing the ball manually
- External forces
- Measurement errors

## Example: Testing Disturbance Rejection

1. **Step PID**: Apply disturbance at t=2s, see how it recovers
2. **One-Shot Trajectory**: Apply disturbance, see if it can adapt (it won't - fixed trajectory)
3. **Trajectory Update**: Apply disturbance, see how it adapts by regenerating trajectories

## Adding to Other Experiments

The same pattern works for `run_step_pid_experiment.py` and `run_oneshot_trajectory_experiment.py`:

1. Import: `from disturbance import create_step_disturbance`
2. Initialize: `self.disturbance = create_step_disturbance(...)`
3. Apply: `position_x = self.disturbance.apply(position_x, current_time)`

## Manual Disturbance (Future)

For manual disturbances (user pushes ball during experiment):
```python
self.disturbance = Disturbance('manual', magnitude=0.05)
# Then call self.disturbance.apply_manual() when user presses a key
```

