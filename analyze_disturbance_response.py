# Analyze why disturbances might not be visible
# The PID controller is correcting disturbances too quickly

print("="*80)
print("WHY DISTURBANCES MIGHT NOT BE VISIBLE")
print("="*80)

# PID gains (from config or defaults)
Kp = 10.0
error_scale = 100.0  # Error is scaled by 100 in PID controller

# Example: 5cm (0.05m) step disturbance
disturbance_magnitude = 0.05  # meters (5cm)

# Calculate PID response
error = disturbance_magnitude * error_scale  # 0.05 * 100 = 5.0
P_term = Kp * error  # 10.0 * 5.0 = 50.0 degrees
output_limit = 15.0  # degrees

print(f"\n1. DISTURBANCE MAGNITUDE: {disturbance_magnitude*100:.1f}cm")
print(f"2. ERROR (scaled): {error:.1f}")
print(f"3. PROPORTIONAL TERM: {P_term:.1f} degrees")
print(f"4. OUTPUT LIMIT: ±{output_limit:.1f} degrees")
print(f"5. ACTUAL OUTPUT: {min(P_term, output_limit):.1f} degrees (clipped)")

print(f"\n{'='*80}")
print("THE PROBLEM:")
print(f"{'='*80}")
print("The PID controller sees the disturbance immediately and generates")
print(f"a large control signal ({min(P_term, output_limit):.1f} degrees) to correct it.")
print("This happens in ONE control cycle (~30ms), so the disturbance")
print("effect is barely visible in the data.")

print(f"\n{'='*80}")
print("SOLUTIONS:")
print(f"{'='*80}")
print("1. LARGER DISTURBANCE: Increase magnitude (e.g., 0.10m = 10cm)")
print("2. LONGER DURATION: Use impulse with longer duration (e.g., 2.0s)")
print("3. APPLY TO ACTUATOR: Disturb the platform tilt directly (more realistic)")
print("4. APPLY AT STEADY STATE: Wait until ball is near 0cm, then disturb")
print("5. CONTINUOUS DISTURBANCE: Use sinusoidal or continuous force")

print(f"\n{'='*80}")
print("RECOMMENDED TEST:")
print(f"{'='*80}")
print("Apply a 10cm impulse disturbance for 1.0 second when ball is at steady state:")
print("  self.disturbance = create_impulse_disturbance(")
print("      time=5.0,  # After system has settled")
print("      magnitude=0.10,  # 10cm (larger)")
print("      duration=1.0  # 1 second (longer)")
print("  )")

