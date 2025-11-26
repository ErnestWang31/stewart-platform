# Test if disturbance is actually being applied
from disturbance import create_impulse_disturbance
import numpy as np

# Create the same disturbance as in the experiment
dist = create_impulse_disturbance(time=0.5, magnitude=0.1, duration=1.0, apply_to='position')

print("Testing disturbance application:")
print(f"Disturbance type: {dist.disturbance_type}")
print(f"Apply to: {dist.apply_to}")
print(f"Params: {dist.params}")

# Test at different times
test_times = [0.0, 0.4, 0.5, 0.6, 1.0, 1.5, 2.0]
position = 0.05  # 5cm

print("\nPosition disturbance test (should add 0.1m = 10cm):")
for t in test_times:
    disturbed = dist.apply(position, t)
    change = disturbed - position
    print(f"  t={t:.1f}s: position={position*100:.1f}cm -> {disturbed*100:.1f}cm (change: {change*100:+.1f}cm)")

print("\nActuator disturbance test (should add 0.1 degrees - but this is wrong, should be degrees):")
for t in test_times:
    control = 0.0
    disturbed = dist.apply_to_actuator(control, t)
    change = disturbed - control
    print(f"  t={t:.1f}s: control={control:.1f}° -> {disturbed:.1f}° (change: {change:+.1f}°)")

print("\n" + "="*60)
print("ISSUE IDENTIFIED:")
print("="*60)
print("When apply_to='position', magnitude=0.1 means 0.1 meters = 10cm")
print("But the PID controller scales error by 100, so:")
print("  Error = 0.1m * 100 = 10.0")
print("  P term = 10.0 * 10.0 = 100 degrees (clipped to 15°)")
print("So the PID should respond VERY aggressively!")
print("\nIf you're not seeing an effect, check:")
print("  1. Is the disturbance actually being applied? (check logs)")
print("  2. Is the disturbance happening at the right time? (t=0.5s)")
print("  3. Is the position being logged correctly? (check CSV)")

