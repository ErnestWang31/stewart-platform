#!/usr/bin/env python3
"""
Step-by-step tuning wizard for the Stewart platform.
Guides you through systematic tuning of all 3 motors.
"""

import json
import os
import math

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
    print("✓ Configuration saved\n")

def print_motor_info(config):
    """Print information about motor configuration."""
    if 'motor_angles_deg' in config:
        angles = config['motor_angles_deg']
        print("Motor Configuration:")
        for i, angle in enumerate(angles, 1):
            cos_val = math.cos(math.radians(angle))
            sin_val = math.sin(math.radians(angle))
            print(f"  Motor {i}: {angle:.1f}° → proj = {cos_val:.3f}*X + {sin_val:.3f}*Y")
        print()

def get_current_gains(config, motor_num):
    """Get current PID gains for a motor."""
    kp = config['pid'].get(f'Kp_m{motor_num}', 0.0)
    ki = config['pid'].get(f'Ki_m{motor_num}', 0.0)
    kd = config['pid'].get(f'Kd_m{motor_num}', 0.0)
    return kp, ki, kd

def set_gains(config, motor_num, kp, ki, kd):
    """Set PID gains for a motor."""
    config['pid'][f'Kp_m{motor_num}'] = kp
    config['pid'][f'Ki_m{motor_num}'] = ki
    config['pid'][f'Kd_m{motor_num}'] = kd

def disable_motor(config, motor_num):
    """Disable a motor (set all gains to 0)."""
    set_gains(config, motor_num, 0.0, 0.0, 0.0)

def tune_motor_interactive(config, motor_num):
    """Interactive tuning for a single motor."""
    print(f"\n{'='*70}")
    print(f"TUNING MOTOR {motor_num}")
    print(f"{'='*70}")
    
    # Get current gains
    kp, ki, kd = get_current_gains(config, motor_num)
    
    print(f"\nCurrent gains: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
    print("\nRecommended starting values:")
    print("  Kp = 1.0-2.0 (start conservative)")
    print("  Ki = 0.0 (add later if needed)")
    print("  Kd = 0.0 (add later if needed)")
    
    # Disable other motors
    for i in range(1, 4):
        if i != motor_num:
            disable_motor(config, i)
    
    # Set starting values if motor is disabled
    if kp == 0.0 and ki == 0.0 and kd == 0.0:
        print(f"\nMotor {motor_num} is currently disabled.")
        use_default = input("Use default starting values? (Kp=1.0, Ki=0, Kd=0) [Y/n]: ").strip().lower()
        if use_default != 'n':
            set_gains(config, motor_num, 1.0, 0.0, 0.0)
            kp, ki, kd = 1.0, 0.0, 0.0
            save_config(config)
    
    print(f"\nMotor {motor_num} is now enabled with Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
    print("Other motors are disabled.")
    print("\nNext steps:")
    print("1. Start the controller: python stewart_platform_controller.py")
    print("2. Use GUI sliders to adjust gains in real-time")
    print("3. Test by moving the ball and observing response")
    print("4. When satisfied, come back here to tune the next motor")
    
    input("\nPress Enter when ready to continue to next motor...")

def main():
    """Main tuning wizard."""
    print("="*70)
    print("STEWART PLATFORM PID TUNING WIZARD")
    print("="*70)
    
    config = load_config()
    if not config:
        return
    
    print_motor_info(config)
    
    print("This wizard will guide you through tuning each motor systematically.")
    print("\nTuning Strategy:")
    print("  1. Tune one motor at a time (others disabled)")
    print("  2. Start with Motor 1 (easiest - pure Y-axis)")
    print("  3. Then Motor 2, then Motor 3")
    print("  4. Finally, fine-tune all together")
    
    input("\nPress Enter to start...")
    
    # Step 1: Motor 1
    tune_motor_interactive(config, 1)
    
    # Step 2: Motor 2
    tune_motor_interactive(config, 2)
    
    # Step 3: Motor 3
    tune_motor_interactive(config, 3)
    
    # Step 4: All together
    print(f"\n{'='*70}")
    print("FINAL STEP: TUNE ALL MOTORS TOGETHER")
    print(f"{'='*70}")
    print("\nAll motors are now enabled.")
    print("Fine-tune gains to optimize coordination between motors.")
    print("\nTips:")
    print("  - If motors fight each other, reduce gains slightly")
    print("  - If response is sluggish, increase Kp")
    print("  - If there's oscillation, increase Kd")
    print("  - Test with diagonal and circular ball movements")
    
    save_config(config)
    print("\n✓ Tuning complete! All motors are enabled.")
    print("Start the controller and test the full system.")

if __name__ == "__main__":
    main()

