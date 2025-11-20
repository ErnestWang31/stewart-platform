#!/usr/bin/env python3
"""
Diagnostic script to check why motors with zero gains are moving.
Checks config, PID initialization, and logs actual behavior.
"""

import json
import sys

def diagnose_config():
    """Check configuration file for motor gain settings."""
    print("="*60)
    print("MOTOR MOVEMENT DIAGNOSTIC")
    print("="*60)
    
    try:
        with open('config_stewart.json', 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"ERROR: Could not load config: {e}")
        return
    
    pid_config = config.get('pid', {})
    
    print("\n1. CONFIG FILE SETTINGS:")
    print("-" * 60)
    
    # Check Motor 1
    print(f"Motor 1:")
    print(f"  Kp_m1: {pid_config.get('Kp_m1', 'NOT SET')}")
    print(f"  Ki_m1: {pid_config.get('Ki_m1', 'NOT SET')}")
    print(f"  Kd_m1: {pid_config.get('Kd_m1', 'NOT SET')}")
    print(f"  Fallback Kp_x: {pid_config.get('Kp_x', 'NOT SET')}")
    
    # Check Motor 2
    print(f"\nMotor 2:")
    print(f"  Kp_m2: {pid_config.get('Kp_m2', 'NOT SET')}")
    print(f"  Ki_m2: {pid_config.get('Ki_m2', 'NOT SET')}")
    print(f"  Kd_m2: {pid_config.get('Kd_m2', 'NOT SET')}")
    print(f"  Fallback Kp_y: {pid_config.get('Kp_y', 'NOT SET')}")
    
    # Check Motor 3
    print(f"\nMotor 3:")
    print(f"  Kp_m3: {pid_config.get('Kp_m3', 'NOT SET')}")
    print(f"  Ki_m3: {pid_config.get('Ki_m3', 'NOT SET')}")
    print(f"  Kd_m3: {pid_config.get('Kd_m3', 'NOT SET')}")
    
    print("\n2. EXPECTED BEHAVIOR:")
    print("-" * 60)
    
    # Simulate how controller initializes motors
    motor1_kp = pid_config.get('Kp_m1', pid_config.get('Kp_x', 0.0))
    motor1_ki = pid_config.get('Ki_m1', pid_config.get('Ki_x', 0.0))
    motor1_kd = pid_config.get('Kd_m1', pid_config.get('Kd_x', 0.0))
    
    motor2_kp = pid_config.get('Kp_m2', 0.0)
    motor2_ki = pid_config.get('Ki_m2', 0.0)
    motor2_kd = pid_config.get('Kd_m2', 0.0)
    
    motor3_kp = pid_config.get('Kp_m3', 0.0)
    motor3_ki = pid_config.get('Ki_m3', 0.0)
    motor3_kd = pid_config.get('Kd_m3', 0.0)
    
    print(f"Motor 1 will use: Kp={motor1_kp}, Ki={motor1_ki}, Kd={motor1_kd}")
    if motor1_kp == 0.0 and motor1_ki == 0.0 and motor1_kd == 0.0:
        print("  WARNING: Motor 1 has ZERO gains - should not move!")
    else:
        print("  OK: Motor 1 has active gains")
    
    print(f"\nMotor 2 will use: Kp={motor2_kp}, Ki={motor2_ki}, Kd={motor2_kd}")
    if motor2_kp == 0.0 and motor2_ki == 0.0 and motor2_kd == 0.0:
        print("  OK: Motor 2 has ZERO gains - should not move")
    else:
        print("  PROBLEM: Motor 2 has active gains - THIS IS THE ISSUE!")
        if pid_config.get('Kp_m2') is None:
            print(f"     Motor 2 is falling back to Kp_y={pid_config.get('Kp_y', 0.0)}")
            print("     FIX: The code now prevents this, but you need to restart the controller!")
    
    print(f"\nMotor 3 will use: Kp={motor3_kp}, Ki={motor3_ki}, Kd={motor3_kd}")
    if motor3_kp == 0.0 and motor3_ki == 0.0 and motor3_kd == 0.0:
        print("  OK: Motor 3 has ZERO gains - should not move")
    else:
        print("  PROBLEM: Motor 3 has active gains - THIS IS THE ISSUE!")
    
    print("\n3. RECOMMENDED FIX:")
    print("-" * 60)
    print("To ensure Motors 2 and 3 don't move, explicitly set in config_stewart.json:")
    print("""
  "pid": {
    "Kp_m1": 1.6,    // Your desired value for Motor 1
    "Ki_m1": 0.0,
    "Kd_m1": 0.0,
    "Kp_m2": 0.0,    // EXPLICITLY set to 0
    "Ki_m2": 0.0,    // EXPLICITLY set to 0
    "Kd_m2": 0.0,    // EXPLICITLY set to 0
    "Kp_m3": 0.0,    // EXPLICITLY set to 0
    "Ki_m3": 0.0,    // EXPLICITLY set to 0
    "Kd_m3": 0.0     // EXPLICITLY set to 0
  }
""")
    
    print("\n4. CHECKING FOR OTHER ISSUES:")
    print("-" * 60)
    
    # Check if there are any other settings that might affect motors
    servo_config = config.get('servo', {})
    motor_direction_invert = servo_config.get('motor_direction_invert', [False, False, False])
    print(f"Motor direction invert: {motor_direction_invert}")
    
    motor_scale_factor = config.get('platform', {}).get('motor_scale_factor', 1.0)
    print(f"Motor scale factor: {motor_scale_factor}")
    if motor_scale_factor != 1.0:
        print("  WARNING: Non-unity scale factor might cause unexpected behavior")
    
    print("\n" + "="*60)

if __name__ == "__main__":
    diagnose_config()

