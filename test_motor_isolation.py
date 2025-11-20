#!/usr/bin/env python3
"""
Test script to isolate motor movement:
- One motor UP (30°)
- Other two motors DOWN (0°)

This helps verify:
1. Each motor responds correctly to commands
2. Motors don't interfere with each other
3. No cross-talk or corruption between motors
"""

import serial
import time
from datetime import datetime

PORT = "COM3"
BAUDRATE = 9600

# Test patterns: [motor1, motor2, motor3]
# 0° = down, 15° = neutral, 30° = up
TEST_PATTERNS = [
    ("Motor 1 UP, others DOWN", [30, 0, 0]),
    ("Motor 2 UP, others DOWN", [0, 30, 0]),
    ("Motor 3 UP, others DOWN", [0, 0, 30]),
    ("All NEUTRAL", [15, 15, 15]),
    ("Motor 1 UP, others NEUTRAL", [30, 15, 15]),
    ("Motor 2 UP, others NEUTRAL", [15, 30, 15]),
    ("Motor 3 UP, others NEUTRAL", [15, 15, 30]),
]

HOLD_TIME_SECONDS = 20  # How long to hold each pattern

def connect_arduino(port, baudrate):
    """Connect to Arduino."""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        print(f"✓ Connected to Arduino on {port}")
        return ser
    except Exception as e:
        print(f"✗ Failed to connect to Arduino: {e}")
        return None

def send_command(ser, angles, description):
    """Send command and verify."""
    command_bytes = bytes(angles)
    
    # Clear buffer
    for _ in range(3):
        if ser.in_waiting > 0:
            ser.reset_input_buffer()
        time.sleep(0.001)
    
    # Send command
    bytes_written = ser.write(command_bytes)
    ser.flush()
    
    if bytes_written != 3:
        print(f"  ⚠ WARNING: Only {bytes_written}/3 bytes sent!")
        return False
    
    # Try to read Arduino response
    time.sleep(0.01)
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line and ("Received bytes:" in line or "CORRUPTION" in line.upper()):
                print(f"  Arduino: {line}")
                if "CORRUPTION" in line.upper():
                    print(f"  ⚠ CORRUPTION DETECTED!")
        except:
            pass
    
    return True

def main():
    """Main test function."""
    print("="*60)
    print("MOTOR ISOLATION TEST")
    print("="*60)
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Hold time per pattern: {HOLD_TIME_SECONDS} seconds")
    print("\nThis test will:")
    print("- Move one motor UP (30°) while others are DOWN (0°)")
    print("- Test each motor individually")
    print("- Verify no cross-talk or corruption")
    print("="*60)
    print("\nStarting test in 3 seconds...")
    time.sleep(3)
    
    ser = connect_arduino(PORT, BAUDRATE)
    if not ser:
        return
    
    try:
        print("\n" + "="*60)
        print("TESTING EACH MOTOR INDIVIDUALLY")
        print("="*60)
        
        # First, go to neutral
        print("\n1. Moving all motors to NEUTRAL...")
        for _ in range(10):
            send_command(ser, [15, 15, 15], "Neutral")
            time.sleep(0.03)
        print("   ✓ All motors at neutral")
        time.sleep(2)
        
        # Test each motor
        for i, (description, angles) in enumerate(TEST_PATTERNS, 2):
            print(f"\n{i}. {description}")
            print(f"   Command: [{angles[0]}, {angles[1]}, {angles[2]}]")
            print(f"   Holding for {HOLD_TIME_SECONDS} seconds...")
            
            # Send command repeatedly for the hold duration
            start_time = time.time()
            command_count = 0
            while time.time() - start_time < HOLD_TIME_SECONDS:
                send_command(ser, angles, description)
                command_count += 1
                time.sleep(0.03)  # 30ms interval
            
            print(f"   ✓ Sent {command_count} commands")
            print(f"   Observe motors - verify correct behavior")
            time.sleep(1)
        
        # Return to neutral
        print("\n" + "="*60)
        print("RETURNING TO NEUTRAL")
        print("="*60)
        print("Moving all motors to neutral...")
        for _ in range(10):
            send_command(ser, [15, 15, 15], "Neutral")
            time.sleep(0.03)
        print("✓ All motors returned to neutral")
        
        print("\n" + "="*60)
        print("TEST COMPLETE")
        print("="*60)
        print("\nWhat to check:")
        print("1. Did each motor move UP when commanded?")
        print("2. Did the other motors stay DOWN/NEUTRAL?")
        print("3. Did motors 2 and 3 move unexpectedly?")
        print("4. Any corruption warnings in Arduino responses?")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
        # Return to neutral
        print("Returning to neutral...")
        for _ in range(10):
            send_command(ser, [15, 15, 15], "Neutral")
            time.sleep(0.03)
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()
        print("\nDone.")

if __name__ == "__main__":
    main()

