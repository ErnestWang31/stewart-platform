# Stewart Platform Ball Balancing System

This folder contains the adapted code for a Stewart platform ball balancing system, converted from the original 1D beam balancer.

## Overview

The system uses computer vision to detect a ping pong ball on a **circular Stewart platform** and uses PID control to balance it at a desired position. The platform is controlled by **3 motors (servos)** arranged at 120° intervals that adjust the platform's roll and pitch angles.

## Files

- **ball_detection_2d.py**: 2D ball detection module that tracks ball position in (x, y) coordinates
- **pid_controller_2d.py**: 2D PID controller with separate controllers for X (roll) and Y (pitch) axes
- **stewart_platform_controller.py**: Main controller integrating ball detection and PID control
- **calibration_2d.py**: Interactive calibration tool for color detection, geometry, and platform limits
- **inverse_kinematics.py**: Inverse kinematics solver for Stewart platform (converts roll/pitch to motor angles)
- **stewart_platform_arduino.ino**: Arduino sketch for controlling 3 servos via Adafruit PWM Servo Driver

## Setup

1. **Install Dependencies**:
   ```bash
   pip install opencv-python numpy matplotlib pyserial scipy
   ```
   
   Note: `pyserial` provides the `serial` module, and `scipy` is needed for the inverse kinematics solver.

2. **Calibration**:
   Run the calibration tool first to configure the system:
   ```bash
   python calibration_2d.py
   ```
   
   This will:
   - Calibrate HSV color bounds for ball detection
   - Set up pixel-to-meter conversion ratio (circular platform uses uniform ratio)
   - Calibrate platform center and radius
   - Find platform position limits
   - Generate `config_stewart.json` configuration file
   
   **Calibration steps:**
   1. Click on the ball multiple times to sample its color, press 'c' when done
   2. Click on the platform center, then click on the platform edge
   3. Click on the 3 motor positions (motor 1, motor 2, motor 3) - this calibrates motor positions relative to the platform
   4. Optionally find limits automatically by tilting the platform (press 'l')
   5. Press 's' to save the configuration
   
   **Motor Calibration**: The system automatically determines motor angles from the clicked positions. This allows the system to work correctly even if the platform is rotated or the motors are wired differently. Click on the visible motor attachment points or markers on your platform in order (motor 1, then motor 2, then motor 3). You can press 'm' to skip motor calibration and use default positions.

3. **Run Controller**:
   ```bash
   python stewart_platform_controller.py
   ```

## Configuration

The system uses `config_stewart.json` for configuration. Key parameters:

- **Camera**: Index, frame width/height
- **Ball Detection**: HSV color bounds
- **Platform**: Type (circular), radius, center position, and radius in pixels
- **Calibration**: Pixel-to-meter conversion ratio (uniform for circular platform)
- **PID**: Separate gains for X (roll) and Y (pitch) axes
- **Servo**: Serial port (COM port) for Arduino, neutral angles for 3 motors, motor direction inversion flags, and calibrated motor positions/angles
- **Platform Limits**: Maximum roll and pitch angles

## How It Works

1. **Ball Detection**: The camera captures video frames, and the ball is detected using HSV color filtering. The ball's (x, y) position is calculated in meters from the platform center.

2. **PID Control**: Two separate PID controllers calculate control outputs:
   - **X-axis controller** outputs **roll angle** (rotation around X axis)
     - Positive roll = platform tilts RIGHT (right side down)
     - Negative roll = platform tilts LEFT (left side down)
   - **Y-axis controller** outputs **pitch angle** (rotation around Y axis)
     - Positive pitch = platform tilts FORWARD (front down)
     - Negative pitch = platform tilts BACKWARD (back down)
   - See `ROLL_AND_PITCH_EXPLANATION.md` for detailed visual explanation

3. **Motor Control**: The roll and pitch angles are converted to motor commands for the 3 servos arranged at 120° intervals around the circular platform. 
   
   **Motor Position Calibration**: During calibration, you click on the 3 motor positions in the camera view. The system automatically calculates the angle of each motor relative to the platform center. This allows the system to work correctly regardless of:
   - Platform rotation relative to the camera
   - Motor wiring differences
   - Physical motor arrangement
   
   The motor positions are saved in the config file as `motor_angles_deg`. If not calibrated, the system uses default positions (90°, 210°, 330°).
   
   The current implementation uses a simplified trigonometric mapping based on the calibrated motor positions. For a real Stewart platform, you would use inverse kinematics (like in `SPV4.py`) to convert platform orientation to motor angles.

4. **Feedback Loop**: The control loop continuously:
   - Detects ball position
   - Calculates error from setpoint
   - Computes PID control output
   - Sends motor commands
   - Repeats

## PID Tuning

The GUI provides real-time PID tuning with sliders:
- **Kp**: Proportional gain (response to current error)
- **Ki**: Integral gain (eliminates steady-state error)
- **Kd**: Derivative gain (damping, reduces overshoot)

Separate gains can be set for X and Y axes.

## Arduino Setup

The system uses a single Arduino with an **Adafruit PWM Servo Driver** shield to control all 3 servos via I2C.

### Hardware Requirements:
- Arduino (Uno, Nano, or compatible)
- Adafruit 16-Channel PWM Servo Driver (PCA9685)
- 3 servos connected to channels 0, 1, and 2 on the PWM driver
- I2C connection between Arduino and PWM driver

### Software Setup:

1. **Install Arduino Libraries**:
   - Install `Adafruit PWM Servo Driver Library` via Arduino Library Manager
   - Install `Wire` library (usually included with Arduino IDE)

2. **Upload Arduino Sketch**:
   - Open `stewart_platform_arduino.ino` in Arduino IDE
   - Select your Arduino board and COM port
   - Upload the sketch to Arduino

3. **Configure Serial Port** (or run calibration to auto-configure):
   - The calibration tool (`calibration_2d.py`) will save motor positions automatically
   - You can manually update `config_stewart.json` with the correct COM port:
     ```json
     "servo": {
       "port": "COM3",  // Change to your Arduino's COM port
       "neutral_angles": [15, 15, 15],
       "motor_direction_invert": [false, false, false],  // Set to true for motors spinning wrong way
       "motor_angles_deg": [90.0, 210.0, 330.0]  // Calibrated motor angles (set by calibration tool)
     }
     ```
   
   **Motor Configuration Notes**:
   - `motor_angles_deg`: Motor angles in degrees from platform center (0° = right, 90° = top, counter-clockwise)
   - These angles are automatically calculated during calibration when you click on motor positions
   - If not set, the system uses default angles (90°, 210°, 330°)
   - The motor calibration accounts for platform rotation and wiring differences

### Arduino Communication Protocol:

- Python sends 3 bytes over serial: `[servo1_angle, servo2_angle, servo3_angle]`
- Each angle is 0-30 degrees (byte value)
- Arduino maps angles to PWM values (280-415) for the servo driver
- Servos are controlled on channels 0, 1, and 2

### Servo Configuration:

- **PWM Range**: 280 (servo up, angle 0°) to 415 (servo down, angle 30°)
- **Neutral Position**: 375 (angle 15°)
- **Frequency**: 50 Hz (standard for analog servos)
- **Angle Range**: 0-30 degrees per servo

### Testing Motor Control:

After uploading the Arduino sketch, test the motors using the test script:

```bash
python test_motors.py
```

The test script provides several test modes:
1. **Run all automated tests** - Comprehensive test suite
2. **Test individual servos** - Test each servo one at a time
3. **Test all servos together** - Move all servos simultaneously
4. **Test sweep motion** - Smooth sweeping motion
5. **Test sequential pattern** - Sequential movement patterns
6. **Custom angle test** - Interactive mode to test specific angles
7. **Quick connection test** - Basic test with Arduino debug output
8. **Continuous monitoring** - Continuous test with real-time Arduino feedback

**Troubleshooting:**
- If motors don't move, check that DEBUG_MODE is enabled in the Arduino sketch
- Verify the COM port is correct (check Device Manager on Windows)
- Ensure no other program is using the serial port
- Check that servos are properly connected to channels 0, 1, and 2
- Verify PWM driver is connected via I2C (SDA/SCL)

## Notes

- The motor control uses inverse kinematics (if enabled) or simplified trigonometric mapping from roll/pitch to motor angles.
- The system uses a single Arduino with PWM Servo Driver instead of multiple serial ports.
- The servo control protocol: Python sends 3 bytes (0-30 degrees each) to Arduino, which converts to PWM signals.
- The system can run in simulation mode if Arduino is not connected.

## Differences from 1D System

1. **2D Ball Detection**: Detects both X and Y position instead of just X
2. **Dual PID Controllers**: Separate controllers for roll and pitch
3. **3 Motor Control**: Controls 3 motors arranged at 120° intervals instead of 1
4. **Circular Platform**: Calibrates circular platform (center + radius) instead of rectangular corners
5. **Uniform Calibration**: Uses uniform pixel-to-meter ratio (circular symmetry)
6. **Platform Geometry**: Circular platform with motors at 120° spacing

