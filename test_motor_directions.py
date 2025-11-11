#!/usr/bin/env python3
"""
Test script to determine which motors need direction inversion
Tests platform tilt commands and identifies incorrect motor directions

Usage:
    python test_motor_directions.py
"""

import sys
import os
import time

# Import controller from same directory
from stewart_platform_controller import StewartPlatformController

def test_motor_directions():
    """Test platform tilt commands to identify incorrect motor directions."""
    
    print("="*60)
    print("Motor Direction Test for Stewart Platform")
    print("="*60)
    print("\nThis test will tilt the platform in specific directions.")
    print("Observe which motors move and in which direction.")
    print("Compare with expected behavior to identify motors that need flipping.")
    print("\nIMPORTANT: Watch the physical platform and note which direction")
    print("each motor moves. You'll need to identify which motors are incorrect.")
    print("\nPress Ctrl+C to stop at any time.\n")
    
    input("Press Enter to start the test (make sure platform is ready)...")
    
    # Initialize controller
    try:
        # Get config file path (same directory as this script)
        config_path = os.path.join(os.path.dirname(__file__), 'config_stewart.json')
        controller = StewartPlatformController(config_file=config_path)
    except Exception as e:
        print(f"Failed to initialize controller: {e}")
        print("Make sure config_stewart.json exists and is valid.")
        return
    
    if not controller.connect_servos():
        print("Failed to connect to servos. Exiting.")
        print("Make sure Arduino is connected and the correct COM port is set in config.")
        return
    
    print("\n✓ Connected to servos successfully")
    print("Starting tests in 2 seconds...\n")
    time.sleep(2)
    
    try:
        # Test 1: Positive Roll
        print("\n" + "="*60)
        print("TEST 1: Roll Test")
        print("="*60)
        print("Command: roll = +10°, pitch = 0°")
        print("\nExpected: Platform should tilt towards Motor 2 side")
        print("(Motor 2 side goes DOWN, Motor 3 side goes UP)")
        print("\nWatch the screen - Motor 2 is ORANGE, Motor 3 is MAGENTA")
        print("The platform should tilt TOWARDS the Motor 2 (orange) side\n")
        
        # Gradual tilt for better visibility
        print("Gradually tilting platform...")
        for step in [2.0, 4.0, 6.0, 8.0, 10.0]:
            controller.send_platform_tilt(roll_angle=step, pitch_angle=0.0)
            time.sleep(0.4)
        print("Platform tilted to 10° roll")
        time.sleep(3)  # Hold for observation
        
        print("\nWhich direction did the platform tilt?")
        result = input("  (1) Towards Motor 1 (yellow) | (2) Towards Motor 2 (orange) | (3) Towards Motor 3 (magenta) | (4) Between motors: ").strip()
        if result == '2':
            print("✓ Correct! Platform tilted towards Motor 2")
        elif result == '3':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor 3 instead of Motor 2")
            print("   Motors 2 and/or 3 may need flipping")
        elif result == '1':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor 1 (should tilt towards Motor 2)")
            print("   Motors 2 and/or 3 may need flipping")
        else:
            print("⚠ Unexpected tilt direction - Motors 2 and/or 3 may need flipping")
        
        input("\nPress Enter to continue to next test...")
        
        # Return to neutral
        print("Returning to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
        time.sleep(2)
        
        # Test 2: Negative Roll
        print("\n" + "="*60)
        print("TEST 2: Roll Test (Opposite)")
        print("="*60)
        print("Command: roll = -10°, pitch = 0°")
        print("\nExpected: Platform should tilt towards Motor 3 side")
        print("(Motor 3 side goes DOWN, Motor 2 side goes UP)")
        print("\nWatch the screen - Motor 2 is ORANGE, Motor 3 is MAGENTA")
        print("The platform should tilt TOWARDS the Motor 3 (magenta) side\n")
        
        # Gradual tilt for better visibility
        print("Gradually tilting platform...")
        for step in [-2.0, -4.0, -6.0, -8.0, -10.0]:
            controller.send_platform_tilt(roll_angle=step, pitch_angle=0.0)
            time.sleep(0.4)
        print("Platform tilted to -10° roll")
        time.sleep(3)  # Hold for observation
        
        print("\nWhich direction did the platform tilt?")
        result = input("  (1) Towards Motor 1 (yellow) | (2) Towards Motor 2 (orange) | (3) Towards Motor 3 (magenta) | (4) Between motors: ").strip()
        if result == '3':
            print("✓ Correct! Platform tilted towards Motor 3")
        elif result == '2':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor 2 instead of Motor 3")
            print("   Motors 2 and/or 3 may need flipping")
        elif result == '1':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor 1 (should tilt towards Motor 3)")
            print("   Motors 2 and/or 3 may need flipping")
        else:
            print("⚠ Unexpected tilt direction - Motors 2 and/or 3 may need flipping")
        
        input("\nPress Enter to continue to next test...")
        
        # Return to neutral
        print("Returning to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
        time.sleep(2)
        
        # Test 3: Positive Pitch
        print("\n" + "="*60)
        print("TEST 3: Pitch Test")
        print("="*60)
        print("Command: roll = 0°, pitch = +10°")
        print("\nExpected: Platform should tilt towards Motor 1")
        print("(Motor 1 side goes DOWN, back side goes UP)")
        print("\nWatch the screen - Motor 1 is YELLOW")
        print("The platform should tilt TOWARDS the Motor 1 (yellow) side\n")
        
        # Gradual tilt for better visibility
        print("Gradually tilting platform...")
        for step in [2.0, 4.0, 6.0, 8.0, 10.0]:
            controller.send_platform_tilt(roll_angle=0.0, pitch_angle=step)
            time.sleep(0.4)
        print("Platform tilted to 10° pitch")
        time.sleep(3)  # Hold for observation
        
        print("\nWhich direction did the platform tilt?")
        result = input("  (1) Towards Motor 1 (yellow) | (2) Towards Motor 2 (orange) | (3) Towards Motor 3 (magenta) | (4) Between motors: ").strip()
        if result == '1':
            print("✓ Correct! Platform tilted towards Motor 1")
        elif result == '2' or result == '3':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor {} instead of Motor 1".format(result))
            print("   Motor 1 may need flipping")
        else:
            print("⚠ Unexpected tilt direction - Motor 1 may need flipping")
        
        input("\nPress Enter to continue to next test...")
        
        # Return to neutral
        print("Returning to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
        time.sleep(2)
        
        # Test 4: Negative Pitch
        print("\n" + "="*60)
        print("TEST 4: Pitch Test (Opposite)")
        print("="*60)
        print("Command: roll = 0°, pitch = -10°")
        print("\nExpected: Platform should tilt AWAY from Motor 1")
        print("(Back side goes DOWN, Motor 1 side goes UP)")
        print("\nWatch the screen - Motor 1 is YELLOW")
        print("The platform should tilt AWAY from Motor 1 (back goes down)\n")
        print("Or: tilt TOWARDS the area between Motor 2 and Motor 3")
        
        # Gradual tilt for better visibility
        print("Gradually tilting platform...")
        for step in [-2.0, -4.0, -6.0, -8.0, -10.0]:
            controller.send_platform_tilt(roll_angle=0.0, pitch_angle=step)
            time.sleep(0.4)
        print("Platform tilted to -10° pitch")
        time.sleep(3)  # Hold for observation
        
        print("\nWhich direction did the platform tilt?")
        result = input("  (1) Towards Motor 1 (yellow) | (2) Towards Motor 2 (orange) | (3) Towards Motor 3 (magenta) | (4) Between motors (away from M1): ").strip()
        if result == '4':
            print("✓ Correct! Platform tilted away from Motor 1 (towards back/between M2 and M3)")
        elif result == '1':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor 1 (should tilt away)")
            print("   Motor 1 may need flipping")
        elif result == '2' or result == '3':
            print("⚠ WRONG DIRECTION - Platform tilted towards Motor {} (should tilt away from M1)".format(result))
            print("   Motor 1 may need flipping")
        else:
            print("⚠ Unexpected tilt direction - Motor 1 may need flipping")
        
        input("\nPress Enter to finish...")
        
        # Return to neutral
        print("Returning to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
        time.sleep(2)
        
        # Summary
        print("\n" + "="*60)
        print("TEST COMPLETE")
        print("="*60)
        print("\nBased on your test results:")
        print("1. If platform tilted towards the WRONG motor for any test,")
        print("   the corresponding motors need direction inversion.")
        print("\n2. Summary of what should happen:")
        print("   - Test 1 (roll +): Should tilt towards Motor 2 (orange)")
        print("   - Test 2 (roll -): Should tilt towards Motor 3 (magenta)")
        print("   - Test 3 (pitch +): Should tilt towards Motor 1 (yellow)")
        print("   - Test 4 (pitch -): Should tilt away from Motor 1 (towards back)")
        print("\n3. To fix, update config_stewart.json:")
        print("   - Go to: servo -> motor_direction_invert")
        print("   - Set value to 'true' for motors that moved incorrectly")
        print("   - Current settings:")
        
        # Show current settings
        try:
            import json
            config_path = os.path.join(os.path.dirname(__file__), 'config_stewart.json')
            with open(config_path, 'r') as f:
                config = json.load(f)
            invert_settings = config.get('servo', {}).get('motor_direction_invert', [False, False, False])
            print(f"   - Motor 1: {invert_settings[0]}")
            print(f"   - Motor 2: {invert_settings[1]}")
            print(f"   - Motor 3: {invert_settings[2]}")
        except Exception as e:
            print(f"   (Could not read current settings: {e})")
        
        print("\n3. After updating config, re-run this test to verify.")
        print("\nNote: Motor positions on screen (M1, M2, M3) should match")
        print("      physical motor positions for accurate identification.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user. Returning to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
        time.sleep(1)
    except Exception as e:
        print(f"\n✗ Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller.servo_serial:
            controller.servo_serial.close()
            print("\nDisconnected from servos.")


def test_individual_motor_response():
    """Test individual motor responses to small tilt commands."""
    
    print("="*60)
    print("Individual Motor Response Test")
    print("="*60)
    print("\nThis test applies small tilt commands to observe individual motor behavior.")
    print("Use this to identify which specific motor moves incorrectly.\n")
    
    input("Press Enter to start...")
    
    try:
        # Get config file path (same directory as this script)
        config_path = os.path.join(os.path.dirname(__file__), 'config_stewart.json')
        controller = StewartPlatformController(config_file=config_path)
    except Exception as e:
        print(f"Failed to initialize controller: {e}")
        return
    
    if not controller.connect_servos():
        print("Failed to connect to servos.")
        return
    
    try:
        # Moderate tilt commands to observe individual motors clearly
        tilt_amount = 8.0  # Increased for better visibility (still safe, under 15° max)
        
        print(f"\nApplying tilt commands ({tilt_amount}°)...")
        print("Observe which motors move and in which direction.\n")
        print(f"NOTE: Using {tilt_amount}° tilt for clear visibility (max safe is 15°)\n")
        
        tests = [
            ("Roll + (towards Motor 2)", tilt_amount, 0.0, "Motor 2 (orange)"),
            ("Roll - (towards Motor 3)", -tilt_amount, 0.0, "Motor 3 (magenta)"),
            ("Pitch + (towards Motor 1)", 0.0, tilt_amount, "Motor 1 (yellow)"),
            ("Pitch - (away from Motor 1)", 0.0, -tilt_amount, "Between M2 and M3 (away from M1)"),
        ]
        
        for test_name, roll, pitch, expected_direction in tests:
            print(f"\n--- {test_name} ---")
            print(f"Roll: {roll}°, Pitch: {pitch}°")
            print(f"Expected: Platform should tilt towards {expected_direction}")
            print("Gradually tilting platform...")
            
            # Gradual tilt for smoother, more observable motion
            steps = 5
            for i in range(1, steps + 1):
                step_roll = roll * (i / steps)
                step_pitch = pitch * (i / steps)
                controller.send_platform_tilt(roll_angle=step_roll, pitch_angle=step_pitch)
                time.sleep(0.3)
            
            print(f"Platform tilted to {roll}° roll, {pitch}° pitch")
            time.sleep(2)  # Hold position for observation
            
            print("\nWhich direction did the platform tilt?")
            direction = input("  (1) Motor 1 | (2) Motor 2 | (3) Motor 3 | (4) Between motors: ").strip()
            print(f"  You selected: {direction}")
            
            input("\nPress Enter for next test...")
            
            # Gradual return to neutral
            print("Returning to neutral...")
            for i in range(steps - 1, -1, -1):
                step_roll = roll * (i / steps)
                step_pitch = pitch * (i / steps)
                controller.send_platform_tilt(roll_angle=step_roll, pitch_angle=step_pitch)
                time.sleep(0.2)
            time.sleep(1)
        
        print("\nTest complete. Return to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
        
    except KeyboardInterrupt:
        print("\nTest interrupted. Returning to neutral...")
        controller.send_platform_tilt(roll_angle=0.0, pitch_angle=0.0)
    finally:
        if controller.servo_serial:
            controller.servo_serial.close()


def main():
    """Main function with menu."""
    print("="*60)
    print("Stewart Platform Motor Direction Test")
    print("="*60)
    print("\nSelect test mode:")
    print("1. Full direction test (recommended)")
    print("2. Individual motor response test")
    print("3. Exit")
    
    try:
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == '1':
            test_motor_directions()
        elif choice == '2':
            test_individual_motor_response()
        elif choice == '3':
            print("Exiting.")
            return
        else:
            print("Invalid choice.")
            
    except KeyboardInterrupt:
        print("\n\nExiting.")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

