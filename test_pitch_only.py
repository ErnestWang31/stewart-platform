#!/usr/bin/env python3
"""
Quick test script to test PITCH motion only (roll = 0)
Oscillates the platform in pitch to visualize the pitch axis
"""

import serial
import time
import numpy as np
import json
import sys

def load_config(config_file="config_stewart.json"):
    """Load configuration from JSON file."""
    try:
        with open(config_file, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Config file {config_file} not found!")
        sys.exit(1)

def pitch_to_motor_angles(pitch_angle, neutral_angles, motor_direction_invert):
    """Convert pitch angle to motor angles (roll = 0).
    
    Args:
        pitch_angle: Pitch angle in degrees (positive = tilt forward)
        neutral_angles: List of 3 neutral angles [m1, m2, m3]
        motor_direction_invert: List of 3 booleans for direction inversion
    
    Returns:
        tuple: (motor1_angle, motor2_angle, motor3_angle) in degrees (0-30)
    """
    # Motor positions in degrees (from +X axis, counter-clockwise)
    motor1_angle_deg = 90   # Top, +Y axis
    motor2_angle_deg = 210  # Bottom-left
    motor3_angle_deg = 330  # Bottom-right
    
    # Convert to radians
    motor1_angle_rad = np.radians(motor1_angle_deg)
    motor2_angle_rad = np.radians(motor2_angle_deg)
    motor3_angle_rad = np.radians(motor3_angle_deg)
    
    # Calculate height changes for each motor (roll = 0, so only pitch term)
    # motor_height = -pitch_angle * sin(motor_angle_rad)
    motor1_height = -pitch_angle * np.sin(motor1_angle_rad)  # -pitch * sin(90°) = -pitch
    motor2_height = -pitch_angle * np.sin(motor2_angle_rad)  # -pitch * sin(210°) = pitch/2
    motor3_height = -pitch_angle * np.sin(motor3_angle_rad)  # -pitch * sin(330°) = pitch/2
    
    # Scale factor (1:1 mapping by default)
    scale_factor = 1.0
    
    # Apply direction inversion
    motor1_dir = -1.0 if motor_direction_invert[0] else 1.0
    motor2_dir = -1.0 if motor_direction_invert[1] else 1.0
    motor3_dir = -1.0 if motor_direction_invert[2] else 1.0
    
    # Calculate motor angles from neutral
    motor1_angle = neutral_angles[0] + motor1_height * scale_factor * motor1_dir
    motor2_angle = neutral_angles[1] + motor2_height * scale_factor * motor2_dir
    motor3_angle = neutral_angles[2] + motor3_height * scale_factor * motor3_dir
    
    # Clip to servo range (0-30 degrees)
    motor1_angle = int(np.clip(motor1_angle, 0, 30))
    motor2_angle = int(np.clip(motor2_angle, 0, 30))
    motor3_angle = int(np.clip(motor3_angle, 0, 30))
    
    return motor1_angle, motor2_angle, motor3_angle

def main():
    """Main test function."""
    print("=" * 60)
    print("PITCH-ONLY TEST - Platform will oscillate in pitch")
    print("=" * 60)
    
    # Load config
    config = load_config()
    
    # Get servo configuration
    servo_config = config.get('servo', {})
    # Handle both 'port' (single) and 'ports' (list) - use first port
    if 'ports' in servo_config and isinstance(servo_config['ports'], list):
        servo_port = servo_config['ports'][0]
    else:
        servo_port = servo_config.get('port', 'COM3')
    
    neutral_angles = servo_config.get('neutral_angles', [15, 15, 15])
    motor_direction_invert = servo_config.get('motor_direction_invert', [False, False, False])
    
    print(f"Config loaded:")
    print(f"  Port: {servo_port}")
    print(f"  Neutral angles: {neutral_angles}")
    print(f"  Motor direction invert: {motor_direction_invert}")
    print()
    
    # Connect to Arduino
    try:
        print(f"Connecting to {servo_port}...")
        ser = serial.Serial(servo_port, 9600, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        ser.reset_input_buffer()
        print("Connected!")
        print()
    except Exception as e:
        print(f"ERROR: Failed to connect to {servo_port}: {e}")
        print("Make sure Arduino is connected and the correct COM port is set in config_stewart.json")
        sys.exit(1)
    
    # Test parameters
    max_pitch = 10.0  # Maximum pitch angle in degrees
    duration = 10.0   # Test duration in seconds
    frequency = 0.5   # Oscillation frequency (Hz)
    
    print(f"Test parameters:")
    print(f"  Max pitch: ±{max_pitch}°")
    print(f"  Duration: {duration} seconds")
    print(f"  Frequency: {frequency} Hz")
    print()
    print("Starting pitch oscillation...")
    print("Watch the platform to see the pitch axis!")
    print("Press Ctrl+C to stop early")
    print()
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < duration:
            # Calculate current pitch angle (sine wave)
            elapsed = time.time() - start_time
            pitch_angle = max_pitch * np.sin(2 * np.pi * frequency * elapsed)
            
            # Convert to motor angles
            m1, m2, m3 = pitch_to_motor_angles(pitch_angle, neutral_angles, motor_direction_invert)
            
            # Send to Arduino
            ser.write(bytes([m1, m2, m3]))
            
            # Print status every 0.5 seconds
            if int(elapsed * 2) != int((elapsed - 0.05) * 2):
                print(f"Pitch: {pitch_angle:+.2f}° -> M1={m1:2d}°, M2={m2:2d}°, M3={m3:2d}°")
            
            time.sleep(0.05)  # 20 Hz update rate
        
        print()
        print("Test complete! Returning to neutral...")
        
        # Return to neutral
        ser.write(bytes(neutral_angles))
        time.sleep(1)
        
    except KeyboardInterrupt:
        print()
        print("Stopped by user. Returning to neutral...")
        ser.write(bytes(neutral_angles))
        time.sleep(1)
    
    finally:
        ser.close()
        print("Disconnected.")
        print()
        print("=" * 60)
        print("Pitch axis should be perpendicular to the oscillation direction")
        print("Positive pitch tilts FORWARD (front down)")
        print("=" * 60)

if __name__ == "__main__":
    main()

