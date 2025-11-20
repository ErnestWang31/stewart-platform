#!/usr/bin/env python3
"""
Simple test: One motor UP, two motors DOWN
Cycles through each motor being UP.
"""

import serial
import time

PORT = "COM3"
BAUDRATE = 9600

# Patterns: [motor1, motor2, motor3]
# 0° = down, 30° = up
PATTERNS = [
    ("Motor 1 UP", [30, 0, 0]),
    ("Motor 2 UP", [0, 30, 0]),
    ("Motor 3 UP", [0, 0, 30]),
]

HOLD_SECONDS = 20

print("="*60)
print("ONE UP, TWO DOWN TEST")
print("="*60)
print(f"Port: {PORT}")
print("Each motor will be UP (30°) while others are DOWN (0°)")
print(f"Hold time: {HOLD_SECONDS} seconds per pattern")
print("\nStarting in 3 seconds...\n")
time.sleep(3)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    print("✓ Connected\n")
    
    # Start at neutral
    print("Moving to neutral...")
    for _ in range(10):
        # Clear buffer before sending
        for _ in range(3):
            if ser.in_waiting > 0:
                ser.reset_input_buffer()
            time.sleep(0.001)
        
        ser.write(bytes([15, 15, 15]))
        ser.flush()
        
        # Read any responses
        time.sleep(0.01)
        if ser.in_waiting > 0:
            try:
                ser.readline().decode('utf-8', errors='ignore')
            except:
                pass
        
        time.sleep(0.03)
    time.sleep(2)
    print("✓ At neutral\n")
    
    # Test each pattern
    for description, angles in PATTERNS:
        print(f"{description}: [{angles[0]}, {angles[1]}, {angles[2]}]")
        print(f"Holding for {HOLD_SECONDS} seconds...")
        
        start = time.time()
        count = 0
        while time.time() - start < HOLD_SECONDS:
            # CRITICAL: Clear buffer aggressively (same as working test_motor_isolation.py)
            for _ in range(3):
                if ser.in_waiting > 0:
                    ser.reset_input_buffer()
                time.sleep(0.001)
            
            # Send command
            bytes_written = ser.write(bytes(angles))
            ser.flush()
            
            if bytes_written != 3:
                print(f"  ⚠ WARNING: Only {bytes_written}/3 bytes sent!")
            
            # CRITICAL: Read Arduino responses to prevent buffer buildup
            # This matches the working test_motor_isolation.py pattern
            time.sleep(0.01)
            if ser.in_waiting > 0:
                try:
                    # Read and discard Arduino debug messages
                    ser.readline().decode('utf-8', errors='ignore')
                except:
                    pass
            
            count += 1
            time.sleep(0.03)
        
        print(f"✓ Sent {count} commands")
        print("Observe motors - verify behavior\n")
        time.sleep(1)
    
    # Return to neutral
    print("Returning to neutral...")
    for _ in range(10):
        # Clear buffer before sending
        for _ in range(3):
            if ser.in_waiting > 0:
                ser.reset_input_buffer()
            time.sleep(0.001)
        
        ser.write(bytes([15, 15, 15]))
        ser.flush()
        
        # Read any responses
        time.sleep(0.01)
        if ser.in_waiting > 0:
            try:
                ser.readline().decode('utf-8', errors='ignore')
            except:
                pass
        
        time.sleep(0.03)
    print("✓ Done")
    
except KeyboardInterrupt:
    print("\n\nInterrupted. Returning to neutral...")
    for _ in range(10):
        # Clear buffer before sending
        for _ in range(3):
            if ser.in_waiting > 0:
                ser.reset_input_buffer()
            time.sleep(0.001)
        
        ser.write(bytes([15, 15, 15]))
        ser.flush()
        
        # Read any responses
        time.sleep(0.01)
        if ser.in_waiting > 0:
            try:
                ser.readline().decode('utf-8', errors='ignore')
            except:
                pass
        
        time.sleep(0.03)
except Exception as e:
    print(f"\nError: {e}")
finally:
    if 'ser' in locals():
        ser.close()

