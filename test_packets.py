#!/usr/bin/env python3
"""
Test script to check raw packets being sent to Arduino
Shows exactly what bytes are transmitted over serial
"""

import serial
import time
import json
import sys

def load_config():
    """Load config file."""
    try:
        with open('config_stewart.json', 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading config: {e}")
        return None

def test_packets():
    """Test sending packets and show raw data."""
    config = load_config()
    if not config:
        return
    
    servo_config = config.get('servo', {})
    port = servo_config.get('port', 'COM3')
    neutral_angles = servo_config.get('neutral_angles', [15, 15, 15])
    
    print("="*60)
    print("Raw Packet Tester")
    print("="*60)
    print(f"Port: {port}")
    print(f"Neutral angles: {neutral_angles}")
    print()
    
    try:
        ser = serial.Serial(port, 9600, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        ser.reset_input_buffer()
        print("✓ Connected to Arduino")
        print()
        
        # Test cases
        test_cases = [
            ("Neutral", [15, 15, 15]),
            ("All minimum", [0, 0, 0]),
            ("All maximum", [30, 30, 30]),
            ("Motor 1 only", [0, 15, 15]),
            ("Motor 2 only", [15, 0, 15]),
            ("Motor 3 only", [15, 15, 0]),
            ("Sweep test", [10, 20, 25]),
        ]
        
        for test_name, angles in test_cases:
            print(f"\n--- {test_name} ---")
            print(f"Input angles: {angles}")
            
            # Clip to valid range
            clipped = [int(max(0, min(30, a))) for a in angles]
            print(f"Clipped angles: {clipped}")
            
            # Create byte array
            command_bytes = bytes(clipped)
            print(f"Raw bytes (decimal): [{command_bytes[0]}, {command_bytes[1]}, {command_bytes[2]}]")
            print(f"Raw bytes (hex): 0x{command_bytes.hex()} (0x{command_bytes[0]:02X} 0x{command_bytes[1]:02X} 0x{command_bytes[2]:02X})")
            print(f"Raw bytes (binary): {bin(command_bytes[0])}, {bin(command_bytes[1])}, {bin(command_bytes[2])}")
            
            # Send to Arduino
            if ser.in_waiting > 0:
                ser.reset_input_buffer()
            
            bytes_written = ser.write(command_bytes)
            ser.flush()
            
            print(f"Bytes written: {bytes_written}/3")
            
            # Wait a bit and check for Arduino response
            time.sleep(0.2)
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                if response.strip():
                    print(f"Arduino response: {response.strip()}")
            
            time.sleep(0.5)
        
        # Return to neutral
        print("\n--- Returning to neutral ---")
        ser.write(bytes(neutral_angles))
        ser.flush()
        time.sleep(0.5)
        
        ser.close()
        print("\n✓ Test complete")
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_packets()

