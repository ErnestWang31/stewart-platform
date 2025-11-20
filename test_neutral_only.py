#!/usr/bin/env python3
"""
Long-running test script to send ONLY neutral commands [15, 15, 15]
This helps isolate whether random motor movement is due to:
- Software bugs (corruption, wrong commands)
- Hardware issues (electrical noise, PWM driver problems)
- Serial communication problems

Run this and observe the motors - they should NEVER move if this is working correctly.
"""

import serial
import time
import json
from datetime import datetime
import sys

# Configuration
PORT = "COM3"
BAUDRATE = 9600
TEST_DURATION_SECONDS = 300  # 5 minutes
COMMAND_INTERVAL_MS = 30  # Send command every 30ms (same as controller)
NEUTRAL_ANGLE = 15

# Statistics
stats = {
    'total_commands_sent': 0,
    'bytes_written_errors': 0,
    'corruption_detected': 0,
    'arduino_responses': 0,
    'unexpected_responses': 0,
    'start_time': None,
    'end_time': None
}

def load_config():
    """Load configuration from config file."""
    try:
        with open('config_stewart.json', 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Warning: Could not load config: {e}")
        return {}

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

def send_neutral_command(ser, log_file):
    """Send neutral command [15, 15, 15] and log everything."""
    command_bytes = bytes([NEUTRAL_ANGLE, NEUTRAL_ANGLE, NEUTRAL_ANGLE])
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
    # Clear input buffer before sending
    for _ in range(3):
        if ser.in_waiting > 0:
            ser.reset_input_buffer()
        time.sleep(0.001)
    
    # Send command
    bytes_written = ser.write(command_bytes)
    ser.flush()
    
    stats['total_commands_sent'] += 1
    
    # Check if all bytes were sent
    if bytes_written != 3:
        error_msg = f"{timestamp} ERROR: Only {bytes_written}/3 bytes sent!"
        print(error_msg)
        log_file.write(error_msg + "\n")
        stats['bytes_written_errors'] += 1
        return False
    
    # Try to read Arduino response
    time.sleep(0.01)  # Small delay for Arduino to respond
    response_lines = []
    if ser.in_waiting > 0:
        start_time = time.time()
        while time.time() - start_time < 0.1:  # Read for up to 100ms
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response_lines.append(line)
                        stats['arduino_responses'] += 1
                except:
                    break
            time.sleep(0.01)
    
    # Check for unexpected responses
    for line in response_lines:
        # Check for corruption warnings
        if "CORRUPTION" in line.upper() or "SUSPICIOUS" in line.upper():
            error_msg = f"{timestamp} CORRUPTION DETECTED: {line}"
            print(error_msg)
            log_file.write(error_msg + "\n")
            stats['corruption_detected'] += 1
        
        # Check if Arduino received something other than [15, 15, 15]
        if "Received bytes:" in line:
            # Parse: "Received bytes: 15, 15, 15"
            try:
                parts = line.split(":")[1].strip().split(",")
                if len(parts) == 3:
                    received = [int(p.strip()) for p in parts]
                    if received != [NEUTRAL_ANGLE, NEUTRAL_ANGLE, NEUTRAL_ANGLE]:
                        error_msg = f"{timestamp} UNEXPECTED: Sent [15,15,15] but Arduino received {received}"
                        print(error_msg)
                        log_file.write(error_msg + "\n")
                        stats['unexpected_responses'] += 1
            except:
                pass
        
        # Log all responses
        log_file.write(f"{timestamp} ARDUINO: {line}\n")
    
    # Log every 100 commands
    if stats['total_commands_sent'] % 100 == 0:
        log_msg = f"{timestamp} Sent {stats['total_commands_sent']} neutral commands"
        print(log_msg)
        log_file.write(log_msg + "\n")
        log_file.flush()
    
    return True

def print_statistics():
    """Print test statistics."""
    duration = (stats['end_time'] - stats['start_time']).total_seconds() if stats['end_time'] else 0
    print("\n" + "="*60)
    print("TEST STATISTICS")
    print("="*60)
    print(f"Duration: {duration:.1f} seconds ({duration/60:.1f} minutes)")
    print(f"Total commands sent: {stats['total_commands_sent']}")
    print(f"Commands per second: {stats['total_commands_sent']/duration:.1f}" if duration > 0 else "N/A")
    print(f"Bytes written errors: {stats['bytes_written_errors']}")
    print(f"Corruption detected: {stats['corruption_detected']}")
    print(f"Unexpected responses: {stats['unexpected_responses']}")
    print(f"Arduino responses received: {stats['arduino_responses']}")
    print("="*60)
    
    if stats['corruption_detected'] > 0 or stats['unexpected_responses'] > 0:
        print("\n⚠ WARNING: Issues detected! Check log file for details.")
    else:
        print("\n✓ No issues detected in software/logs.")
        print("  If motors still moved, it's likely a hardware issue:")
        print("  - Electrical noise on PWM lines")
        print("  - PWM driver board problems")
        print("  - Servo control line interference")

def main():
    """Main test function."""
    print("="*60)
    print("NEUTRAL COMMAND TEST")
    print("="*60)
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Test duration: {TEST_DURATION_SECONDS} seconds ({TEST_DURATION_SECONDS/60:.1f} minutes)")
    print(f"Command interval: {COMMAND_INTERVAL_MS}ms")
    print(f"Command: [{NEUTRAL_ANGLE}, {NEUTRAL_ANGLE}, {NEUTRAL_ANGLE}] (neutral)")
    print("\nThis test sends ONLY neutral commands.")
    print("Motors should NEVER move during this test.")
    print("="*60)
    print("\nStarting test in 3 seconds...")
    time.sleep(3)
    
    # Connect to Arduino
    ser = connect_arduino(PORT, BAUDRATE)
    if not ser:
        print("Failed to connect. Exiting.")
        return
    
    # Create log file
    log_filename = f"logs/neutral_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    try:
        import os
        os.makedirs('logs', exist_ok=True)
    except:
        pass
    
    stats['start_time'] = datetime.now()
    
    try:
        with open(log_filename, 'w') as log_file:
            log_file.write(f"Neutral Command Test Started: {stats['start_time']}\n")
            log_file.write(f"Port: {PORT}, Baudrate: {BAUDRATE}\n")
            log_file.write(f"Test duration: {TEST_DURATION_SECONDS} seconds\n")
            log_file.write(f"Command: [{NEUTRAL_ANGLE}, {NEUTRAL_ANGLE}, {NEUTRAL_ANGLE}]\n")
            log_file.write("="*60 + "\n\n")
            
            start_time = time.time()
            command_count = 0
            
            print("\nTest running... (Press Ctrl+C to stop early)")
            print("Observe the motors - they should stay completely still.\n")
            
            while time.time() - start_time < TEST_DURATION_SECONDS:
                send_neutral_command(ser, log_file)
                command_count += 1
                
                # Sleep to match controller timing
                time.sleep(COMMAND_INTERVAL_MS / 1000.0)
                
                # Print progress every 10 seconds
                elapsed = time.time() - start_time
                if command_count % (10000 // COMMAND_INTERVAL_MS) == 0:  # ~every 10 seconds
                    remaining = TEST_DURATION_SECONDS - elapsed
                    print(f"Progress: {elapsed:.1f}s / {TEST_DURATION_SECONDS}s ({elapsed/TEST_DURATION_SECONDS*100:.1f}%) | "
                          f"Remaining: {remaining:.1f}s | Commands: {stats['total_commands_sent']}")
            
            stats['end_time'] = datetime.now()
            log_file.write(f"\nTest completed: {stats['end_time']}\n")
            log_file.write("="*60 + "\n")
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
        stats['end_time'] = datetime.now()
    except Exception as e:
        print(f"\n\nError during test: {e}")
        import traceback
        traceback.print_exc()
        stats['end_time'] = datetime.now()
    finally:
        ser.close()
        print(f"\nLog file saved: {log_filename}")
        print_statistics()

if __name__ == "__main__":
    main()

