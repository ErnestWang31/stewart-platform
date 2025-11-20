#!/usr/bin/env python3
"""
Interactive tuning script for a single motor at a time.
This helps you tune each motor independently before tuning all together.

Usage:
1. Run this script
2. Select which motor to tune (1, 2, or 3)
3. Other motors will be disabled (gains = 0)
4. Adjust gains in real-time using the GUI
5. Test by moving the ball and observing response
"""

import json
import sys
import os

def load_config():
    """Load current configuration."""
    config_file = 'config_stewart.json'
    if not os.path.exists(config_file):
        print(f"Error: {config_file} not found!")
        return None
    
    with open(config_file, 'r') as f:
        return json.load(f)

def save_config(config):
    """Save configuration."""
    with open('config_stewart.json', 'w') as f:
        json.dump(config, f, indent=2)
    print("✓ Configuration saved")

def disable_motors(config, keep_motor):
    """Disable all motors except the one being tuned.
    
    Args:
        config: Configuration dict
        keep_motor: Motor number to keep enabled (1, 2, or 3)
    """
    for i in range(1, 4):
        if i != keep_motor:
            config['pid'][f'Kp_m{i}'] = 0.0
            config['pid'][f'Ki_m{i}'] = 0.0
            config['pid'][f'Kd_m{i}'] = 0.0
            print(f"  Motor {i}: Disabled (Kp=0, Ki=0, Kd=0)")
        else:
            # Set starting values if not already set
            if f'Kp_m{i}' not in config['pid']:
                config['pid'][f'Kp_m{i}'] = 1.0
            if f'Ki_m{i}' not in config['pid']:
                config['pid'][f'Ki_m{i}'] = 0.0
            if f'Kd_m{i}' not in config['pid']:
                config['pid'][f'Kd_m{i}'] = 0.0
            print(f"  Motor {i}: Enabled (Kp={config['pid'][f'Kp_m{i}']:.2f}, "
                  f"Ki={config['pid'][f'Ki_m{i}']:.2f}, Kd={config['pid'][f'Kd_m{i}']:.2f})")

def get_motor_info(motor_num):
    """Get information about what each motor affects."""
    motor_angles = {
        1: 90,   # Up (Y-axis primary)
        2: 210,  # Down-left (X-Y combined)
        3: 330   # Down-right (X-Y combined)
    }
    
    angle = motor_angles.get(motor_num, 0)
    import math
    cos_val = math.cos(math.radians(angle))
    sin_val = math.sin(math.radians(angle))
    
    info = {
        1: "Motor 1 (90°): Primarily affects Y-axis (up/down). Best for tuning Y-axis response.",
        2: f"Motor 2 (210°): Affects both X and Y. Projection: {cos_val:.3f}*X + {sin_val:.3f}*Y",
        3: f"Motor 3 (330°): Affects both X and Y. Projection: {cos_val:.3f}*X + {sin_val:.3f}*Y"
    }
    
    return info.get(motor_num, "Unknown motor")

def main():
    """Main tuning function."""
    print("="*70)
    print("SINGLE MOTOR TUNING HELPER")
    print("="*70)
    print("\nThis script helps you tune one motor at a time.")
    print("Other motors will be disabled (gains = 0) so you can focus on one.")
    print("\nRecommended tuning order:")
    print("  1. Motor 1 (easiest - pure Y-axis)")
    print("  2. Motor 2 (X-Y combined)")
    print("  3. Motor 3 (X-Y combined)")
    print("  4. Then tune all together\n")
    
    # Load config
    config = load_config()
    if not config:
        return
    
    # Show current motor angles
    if 'motor_angles_deg' in config:
        angles = config['motor_angles_deg']
        print("Current motor positions:")
        for i, angle in enumerate(angles, 1):
            print(f"  Motor {i}: {angle:.1f}°")
        print()
    
    # Select motor to tune
    while True:
        try:
            motor_str = input("Which motor to tune? (1, 2, or 3, or 'q' to quit): ").strip()
            if motor_str.lower() == 'q':
                print("Exiting without changes.")
                return
            
            motor_num = int(motor_str)
            if motor_num not in [1, 2, 3]:
                print("Invalid motor number. Please enter 1, 2, or 3.")
                continue
            break
        except ValueError:
            print("Invalid input. Please enter 1, 2, or 3.")
        except KeyboardInterrupt:
            print("\nExiting.")
            return
    
    # Show motor info
    print(f"\n{get_motor_info(motor_num)}")
    print(f"\nDisabling other motors and preparing Motor {motor_num} for tuning...")
    
    # Disable other motors
    disable_motors(config, motor_num)
    
    # Save config
    save_config(config)
    
    print("\n" + "="*70)
    print("TUNING INSTRUCTIONS")
    print("="*70)
    print(f"\n1. Start the controller: python stewart_platform_controller.py")
    print(f"2. Motor {motor_num} is enabled, others are disabled")
    print(f"3. Use the GUI sliders to adjust Motor {motor_num} gains:")
    print(f"   - Start with Kp=1.0, Ki=0, Kd=0")
    print(f"   - Increase Kp if response is too slow")
    print(f"   - Add Kd if there's oscillation (start with Kd = Kp * 0.5)")
    print(f"   - Add Ki if there's steady-state error (start with Ki = Kp * 0.05)")
    print(f"\n4. Test by moving the ball and observing response")
    print(f"5. When satisfied, run this script again to tune the next motor")
    print("\n" + "="*70)

if __name__ == "__main__":
    main()

