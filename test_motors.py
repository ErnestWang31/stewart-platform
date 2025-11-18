#!/usr/bin/env python3
"""
Test script for Stewart Platform motors
Tests communication with Arduino and servo movement
"""

import serial
import time
import sys
import json

class MotorTester:
    def __init__(self, port="COM3", baudrate=9600):
        """Initialize motor tester with serial connection."""
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.neutral = 15  # Neutral angle (0-30 range)
        
    def connect(self):
        """Connect to Arduino."""
        try:
            print(f"Connecting to Arduino on {self.port} at {self.baudrate} baud...")
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2.5)  # Wait for Arduino to initialize (Arduino may reset on connection)
            

            
            # Read any initial messages from Arduino (like "Stewart Platform Arduino Ready")
            print("Waiting for Arduino initialization...")

            time.sleep(0.5)
            messages = []
            start_time = time.time()
            while time.time() - start_time < 1.0:  # Read for up to 1 second
                if self.serial_conn.in_waiting > 0:
                    try:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            messages.append(line)
                            print(f"  Arduino: {line}")
                    except:
                        break
                time.sleep(0.1)
            
            if not messages:
                print("  (No initial message from Arduino - this is OK if DEBUG_MODE is off)")
            
            print(f"✓ Connected successfully to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ Connection failed: {e}")
            print(f"  Make sure Arduino is connected to {self.port}")
            print(f"  Check Device Manager (Windows) or ls /dev/tty* (Linux/Mac)")
            print(f"  Close any other programs using the serial port (Arduino IDE Serial Monitor, etc.)")
            return False
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def send_angles(self, angle1, angle2, angle3, wait_for_response=False):
        """Send three servo angles to Arduino.
        
        Args:
            angle1, angle2, angle3: Servo angles in degrees (0-30 range)
            wait_for_response: If True, wait a bit and check for serial response
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("✗ Not connected to Arduino")
            return False
        
        # Clip angles to valid range
        angle1 = max(0, min(30, int(angle1)))
        angle2 = max(0, min(30, int(angle2)))
        angle3 = max(0, min(30, int(angle3)))
        
        try:
            # CRITICAL FIX: Clear input buffer periodically to prevent Arduino debug messages from filling it up
            if self.serial_conn.in_waiting > 50:  # If buffer has accumulated data
                self.serial_conn.reset_input_buffer()
            
            # Send 3 bytes atomically
            command = bytes([angle1, angle2, angle3])
            bytes_written = self.serial_conn.write(command)
            # Remove flush() - it can block and isn't necessary
            
            if bytes_written != 3:
                print(f"⚠ Warning: Only {bytes_written} of 3 bytes sent")
            
            print(f"  Sent: Servo1={angle1}°, Servo2={angle2}°, Servo3={angle3}° (hex: {command.hex()})")
            
            # Wait a bit and check for any response from Arduino (if debug mode is on)
            if wait_for_response:
                time.sleep(0.15)  # Give Arduino time to process and respond
                response_lines = []
                start_time = time.time()
                while time.time() - start_time < 0.2:  # Read for up to 200ms
                    if self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                response_lines.append(line)
                        except:
                            break
                    time.sleep(0.01)
                
                if response_lines:
                    for line in response_lines:
                        print(f"  Arduino: {line}")
            
            return True
        except Exception as e:
            print(f"✗ Failed to send command: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def test_individual_servos(self):
        """Test each servo individually."""
        print("\n" + "="*60)
        print("TEST 1: Individual Servo Test")
        print("="*60)
        print("Testing each servo one at a time...")
        
        # Test Servo 1
        print("\n--- Servo 1 Test ---")
        print("Moving Servo 1 to minimum (0°)...")
        self.send_angles(0, self.neutral, self.neutral)
        time.sleep(1)
        print("Moving Servo 1 to maximum (30°)...")
        self.send_angles(30, self.neutral, self.neutral)
        time.sleep(1)
        print("Moving Servo 1 to neutral (15°)...")
        self.send_angles(self.neutral, self.neutral, self.neutral)
        time.sleep(1)
        
        # Test Servo 2
        print("\n--- Servo 2 Test ---")
        print("Moving Servo 2 to minimum (0°)...")
        self.send_angles(self.neutral, 0, self.neutral)
        time.sleep(1)
        print("Moving Servo 2 to maximum (30°)...")
        self.send_angles(self.neutral, 30, self.neutral)
        time.sleep(1)
        print("Moving Servo 2 to neutral (15°)...")
        self.send_angles(self.neutral, self.neutral, self.neutral)
        time.sleep(1)
        
        # Test Servo 3
        print("\n--- Servo 3 Test ---")
        print("Moving Servo 3 to minimum (0°)...")
        self.send_angles(self.neutral, self.neutral, 0)
        time.sleep(1)
        print("Moving Servo 3 to maximum (30°)...")
        self.send_angles(self.neutral, self.neutral, 30)
        time.sleep(1)
        print("Moving Servo 3 to neutral (15°)...")
        self.send_angles(self.neutral, self.neutral, self.neutral)
        time.sleep(1)
        
        print("\n✓ Individual servo test complete")
    
    def test_all_servos_together(self):
        """Test all servos moving together."""
        print("\n" + "="*60)
        print("TEST 2: All Servos Together Test")
        print("="*60)
        print("Moving all servos together...")
        
        # Move all to minimum
        print("Moving all servos to minimum (0°)...")
        self.send_angles(0, 0, 0)
        time.sleep(1)
        
        # Move all to maximum
        print("Moving all servos to maximum (30°)...")
        self.send_angles(30, 30, 30)
        time.sleep(1)
        
        # Move all to neutral
        print("Moving all servos to neutral (15°)...")
        self.send_angles(self.neutral, self.neutral, self.neutral)
        time.sleep(1)
        
        print("\n✓ All servos test complete")
    
    def test_sweep_motion(self):
        """Test smooth sweep motion."""
        print("\n" + "="*60)
        print("TEST 3: Sweep Motion Test")
        print("="*60)
        print("Sweeping servos from 0° to 30° and back...")
        
        # Sweep up
        for angle in range(0, 31, 2):
            self.send_angles(angle, angle, angle)
            time.sleep(0.1)
        
        # Sweep down
        for angle in range(30, -1, -2):
            self.send_angles(angle, angle, angle)
            time.sleep(0.1)
        
        # Return to neutral
        self.send_angles(self.neutral, self.neutral, self.neutral)
        time.sleep(0.5)
        
        print("\n✓ Sweep motion test complete")
    
    def test_sequential_pattern(self):
        """Test sequential pattern (like the prototype code)."""
        print("\n" + "="*60)
        print("TEST 4: Sequential Pattern Test")
        print("="*60)
        print("Testing sequential movement pattern...")
        
        # Pattern: Move each servo in sequence
        patterns = [
            (0, self.neutral, self.neutral),   # Servo 1 min
            (30, self.neutral, self.neutral),  # Servo 1 max
            (self.neutral, 0, self.neutral),   # Servo 2 min
            (self.neutral, 30, self.neutral),  # Servo 2 max
            (self.neutral, self.neutral, 0),   # Servo 3 min
            (self.neutral, self.neutral, 30),  # Servo 3 max
            (0, 0, 0),                         # All min
            (30, 30, 30),                      # All max
            (self.neutral, self.neutral, self.neutral),  # All neutral
        ]
        
        for i, (a1, a2, a3) in enumerate(patterns, 1):
            print(f"Pattern {i}/{len(patterns)}: Servo1={a1}°, Servo2={a2}°, Servo3={a3}°")
            self.send_angles(a1, a2, a3)
            time.sleep(1)
        
        print("\n✓ Sequential pattern test complete")
    
    def test_custom_angles(self):
        """Interactive test for custom angles."""
        print("\n" + "="*60)
        print("TEST 5: Custom Angle Test")
        print("="*60)
        print("Enter custom angles for each servo (0-30 degrees)")
        print("Type 'q' to quit, 'n' for neutral")
        
        while True:
            try:
                user_input = input("\nEnter angles (servo1 servo2 servo3) or 'q' to quit: ").strip()
                
                if user_input.lower() == 'q':
                    break
                elif user_input.lower() == 'n':
                    self.send_angles(self.neutral, self.neutral, self.neutral)
                    continue
                
                angles = [int(x) for x in user_input.split()]
                if len(angles) != 3:
                    print("Please enter exactly 3 angles")
                    continue
                
                self.send_angles(angles[0], angles[1], angles[2])
                
            except ValueError:
                print("Invalid input. Please enter 3 numbers (0-30)")
            except KeyboardInterrupt:
                break
        
        # Return to neutral
        self.send_angles(self.neutral, self.neutral, self.neutral)
        print("\n✓ Custom angle test complete")
    
    def run_all_tests(self):
        """Run all automated tests."""
        if not self.connect():
            return False
        
        try:
            # Run all tests
            self.test_individual_servos()
            time.sleep(1)
            
            self.test_all_servos_together()
            time.sleep(1)
            
            self.test_sweep_motion()
            time.sleep(1)
            
            self.test_sequential_pattern()
            time.sleep(1)
            
            # Return to neutral
            print("\nReturning all servos to neutral...")
            self.send_angles(self.neutral, self.neutral, self.neutral)
            
            print("\n" + "="*60)
            print("ALL TESTS COMPLETE")
            print("="*60)
            
            return True
            
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
            # Return to neutral
            self.send_angles(self.neutral, self.neutral, self.neutral)
            return False
        except Exception as e:
            print(f"\n✗ Error during testing: {e}")
            return False
        finally:
            self.disconnect()
    
    def disconnect(self):
        """Disconnect from Arduino."""
        if self.serial_conn and self.serial_conn.is_open:
            # Return to neutral before disconnecting
            self.send_angles(self.neutral, self.neutral, self.neutral)
            time.sleep(0.5)
            self.serial_conn.close()
            print(f"\nDisconnected from {self.port}")


def load_port_from_config():
    """Load COM port from config file."""
    try:
        with open('config_stewart.json', 'r') as f:
            config = json.load(f)
            return config.get('servo', {}).get('port', 'COM3')
    except Exception:
        return 'COM3'


def main():
    """Main function."""
    print("="*60)
    print("Stewart Platform Motor Test Script")
    print("="*60)
    
    # Try to load port from config
    port = load_port_from_config()
    print(f"Using port from config: {port}")
    print("(You can override by passing port as argument: python test_motors.py COM4)")
    
    # Allow port override from command line
    if len(sys.argv) > 1:
        port = sys.argv[1]
        print(f"Overriding with port: {port}")
    
    tester = MotorTester(port=port)
    
    if not tester.connect():
        print("\nFailed to connect. Please check:")
        print("1. Arduino is connected and powered on")
        print("2. Correct COM port (check Device Manager)")
        print("3. Arduino sketch is uploaded and running")
        print("4. No other program is using the serial port")
        return
    
    print("\nSelect test mode:")
    print("1. Run all automated tests")
    print("2. Test individual servos")
    print("3. Test all servos together")
    print("4. Test sweep motion")
    print("5. Test sequential pattern")
    print("6. Custom angle test (interactive)")
    print("7. Quick connection test (with debug output)")
    print("8. Continuous monitoring test (with Arduino debug)")
    
    try:
        choice = input("\nEnter choice (1-8): ").strip()
        
        if choice == '1':
            tester.run_all_tests()
        elif choice == '2':
            tester.test_individual_servos()
            tester.disconnect()
        elif choice == '3':
            tester.test_all_servos_together()
            tester.disconnect()
        elif choice == '4':
            tester.test_sweep_motion()
            tester.disconnect()
        elif choice == '5':
            tester.test_sequential_pattern()
            tester.disconnect()
        elif choice == '6':
            tester.test_custom_angles()
            tester.disconnect()
        elif choice == '7':
            print("\nQuick connection test with debugging...")
            print("Testing communication (checking for Arduino debug messages)...")
            tester.send_angles(15, 15, 15, wait_for_response=True)
            time.sleep(0.5)
            print("\nMoving Servo 1...")
            tester.send_angles(0, 15, 15, wait_for_response=True)
            time.sleep(0.5)
            tester.send_angles(30, 15, 15, wait_for_response=True)
            time.sleep(0.5)
            print("\nReturning to neutral...")
            tester.send_angles(15, 15, 15, wait_for_response=True)
            print("✓ Quick test complete")
            tester.disconnect()
        elif choice == '8':
            # New: Continuous monitoring test
            print("\nContinuous monitoring test (press Ctrl+C to stop)...")
            print("Sending commands every 0.5 seconds, monitoring Arduino responses...")
            try:
                angle = 0
                direction = 1
                while True:
                    tester.send_angles(angle, 15, 15, wait_for_response=True)
                    time.sleep(0.5)
                    angle += direction * 5
                    if angle >= 30:
                        angle = 30
                        direction = -1
                    elif angle <= 0:
                        angle = 0
                        direction = 1
            except KeyboardInterrupt:
                print("\nStopping...")
                tester.send_angles(15, 15, 15)
                tester.disconnect()
        else:
            print("Invalid choice")
            tester.disconnect()
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        tester.disconnect()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        tester.disconnect()


if __name__ == "__main__":
    main()

