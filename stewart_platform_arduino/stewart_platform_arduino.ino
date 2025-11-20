#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo PWM configuration (from prototype code)
#define SERVOMIN  280   // min pulse (out of 4096) servo going up
#define SERVOMAX  415   // max pulse (out of 4096) servo going down
#define SERVO_FREQ 50   // 50 Hz for analog servos
#define ZERO_POS  375   // midpoint between 280 and 415 ≈ neutral

// Servo channel assignments (0, 1, 2 on the PWM driver)
const uint8_t SERVO_CH[3] = {0, 15, 7};

// Servo angle configuration
#define MIN_ANGLE 0     // Minimum angle in degrees (0-30 range)
#define MAX_ANGLE 30    // Maximum angle in degrees (0-30 range)
#define NEUTRAL_ANGLE 15 // Neutral position in degrees

// Communication variables
int targetAngles[3] = {NEUTRAL_ANGLE, NEUTRAL_ANGLE, NEUTRAL_ANGLE};
bool newCommand = false;

// Debug mode - set to true to enable serial debugging
#define DEBUG_MODE false

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial port to connect (useful for USB)
  while (!Serial && millis() < 3000) {
    ; // wait for serial port to connect (max 3 seconds)
  }
  
  // Initialize PWM driver
  if (!pwm.begin()) {
    Serial.println("ERROR: PWM driver initialization failed!");
    while (1) delay(10); // Halt if PWM driver fails
  }
  
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Initialize all servos to neutral position
  for (int i = 0; i < 3; i++) {
    int pwmValue = angleToPWM(NEUTRAL_ANGLE);
    pwm.setPWM(SERVO_CH[i], 0, pwmValue);
    targetAngles[i] = NEUTRAL_ANGLE;
    if (DEBUG_MODE) {
      Serial.print("Servo ");
      Serial.print(i);
      Serial.print(" initialized to angle ");
      Serial.print(NEUTRAL_ANGLE);
      Serial.print(" (PWM: ");
      Serial.print(pwmValue);
      Serial.println(")");
    }
  }
  
  // Send ready message
#if DEBUG_MODE
  Serial.println("Stewart Platform Arduino Ready");
  Serial.println("Waiting for 3 bytes: [servo1_angle, servo2_angle, servo3_angle]");
  Serial.println("DEBUG MODE: Enabled");
#endif
}

void loop() {
  // Check for incoming serial data
  // Expected format: 3 bytes (servo1_angle, servo2_angle, servo3_angle)
  // Each angle is 0-30 degrees
  if (Serial.available() >= 3) {
    // Read 3 bytes for the 3 servos
    int angle1 = Serial.read();
    int angle2 = Serial.read();
    int angle3 = Serial.read();
    
    if (DEBUG_MODE) {
      Serial.print("Received bytes: ");
      Serial.print(angle1);
      Serial.print(", ");
      Serial.print(angle2);
      Serial.print(", ");
      Serial.println(angle3);
    }
    
    // Validate angle range (0-30)
    if (angle1 >= MIN_ANGLE && angle1 <= MAX_ANGLE &&
        angle2 >= MIN_ANGLE && angle2 <= MAX_ANGLE &&
        angle3 >= MIN_ANGLE && angle3 <= MAX_ANGLE) {
      
      targetAngles[0] = angle1;
      targetAngles[1] = angle2;
      targetAngles[2] = angle3;
      newCommand = true;
      
      if (DEBUG_MODE) {
        Serial.print("Valid angles: Servo1=");
        Serial.print(angle1);
        Serial.print("°, Servo2=");
        Serial.print(angle2);
        Serial.print("°, Servo3=");
        Serial.print(angle3);
        Serial.println("°");
      }
    } else {
      if (DEBUG_MODE) {
        Serial.print("ERROR: Invalid angle range! Angles must be 0-30. Received: ");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.print(angle2);
        Serial.print(", ");
        Serial.println(angle3);
      }
    }
    
    // Clear any remaining bytes in buffer to prevent confusion
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  
  // Update servos if new command received
  if (newCommand) {
    for (int i = 0; i < 3; i++) {
      int pwmValue = angleToPWM(targetAngles[i]);
      pwm.setPWM(SERVO_CH[i], 0, pwmValue);
      
      if (DEBUG_MODE) {
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(": angle=");
        Serial.print(targetAngles[i]);
        Serial.print("° -> PWM=");
        Serial.println(pwmValue);
      }
    }
    newCommand = false;
    
    if (DEBUG_MODE) {
      Serial.println("---");
    }
  }
  
  // Small delay for stability
  delay(10);
}

/**
 * Convert servo angle (0-30 degrees) to PWM value (280-415)
 * Maps linearly from angle range to PWM range
 * 
 * @param angle Servo angle in degrees (0-30)
 * @return PWM value (280-415)
 */
int angleToPWM(int angle) {
  // Clamp angle to valid range
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
  
  // Linear mapping: 
  // angle 0 (minimum) -> SERVOMIN (280) = servo up
  // angle 15 (neutral) -> ~ZERO_POS (375) = midpoint
  // angle 30 (maximum) -> SERVOMAX (415) = servo down
  int pwmValue = map(angle, MIN_ANGLE, MAX_ANGLE, SERVOMIN, SERVOMAX);
  
  return pwmValue;
}
