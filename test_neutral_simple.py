#!/usr/bin/env python3
"""
Simple infinite test - sends ONLY [15, 15, 15] forever.
Run this and let it go - press Ctrl+C to stop.
"""

import serial
import time
from datetime import datetime

PORT = "COM3"
BAUDRATE = 9600
NEUTRAL = bytes([15, 15, 15])

print("="*60)
print("SIMPLE NEUTRAL TEST - Runs until Ctrl+C")
print("="*60)
print(f"Port: {PORT}")
print("Sending: [15, 15, 15] every 30ms")
print("\nMotors should NEVER move. Press Ctrl+C to stop.\n")

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    print("✓ Connected. Starting test...\n")
    
    count = 0
    start_time = time.time()
    
    while True:
        # Clear buffer
        if ser.in_waiting > 0:
            ser.reset_input_buffer()
        
        # Send neutral
        ser.write(NEUTRAL)
        ser.flush()
        count += 1
        
        # Print every 100 commands (~3 seconds)
        if count % 100 == 0:
            elapsed = time.time() - start_time
            print(f"[{elapsed:6.1f}s] Sent {count} commands | Rate: {count/elapsed:.1f} cmd/s")
        
        time.sleep(0.03)  # 30ms interval
        
except KeyboardInterrupt:
    elapsed = time.time() - start_time
    print(f"\n\nTest stopped.")
    print(f"Total commands sent: {count}")
    print(f"Duration: {elapsed:.1f} seconds")
    print(f"Average rate: {count/elapsed:.1f} commands/second")
except Exception as e:
    print(f"\nError: {e}")
finally:
    if 'ser' in locals():
        ser.close()
    print("Done.")

