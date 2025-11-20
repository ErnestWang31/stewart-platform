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
const uint8_t SERVO_CH[3] = {0, 7, 15};

// Servo angle configuration
#define MIN_ANGLE 0     // Minimum angle in degrees (0-30 range)
#define MAX_ANGLE 30    // Maximum angle in degrees (0-30 range)
#define NEUTRAL_ANGLE 15 // Neutral position in degrees

// Communication variables
int targetAngles[3] = {NEUTRAL_ANGLE, NEUTRAL_ANGLE, NEUTRAL_ANGLE};
bool newCommand = false;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000; // If no command for 1 second, assume connection lost

// Motor lock mechanism - if a motor receives NEUTRAL_ANGLE for this many consecutive commands,
// it will be locked at neutral to prevent accidental movement
const int LOCK_THRESHOLD = 5;  // Lock after 5 consecutive neutral commands
int consecutiveNeutralCount[3] = {0, 0, 0};  // Track consecutive neutral commands per motor
bool motorLocked[3] = {false, false, false};  // Lock state for each motor

// HARD LOCK: If a motor is hard-locked, it will NEVER accept any value other than NEUTRAL_ANGLE
// This is more aggressive than the soft lock - it completely prevents movement
bool motorHardLocked[3] = {false, false, false};  // Hard lock state (can only be set, never cleared automatically)

// Debug mode - set to true to enable serial debugging
#define DEBUG_MODE true

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
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
  Serial.println("Stewart Platform Arduino Ready");
  Serial.println("Waiting for 3 bytes: [servo1_angle, servo2_angle, servo3_angle]");
  if (DEBUG_MODE) {
    Serial.println("DEBUG MODE: Enabled");
  }
}

void loop() {
  // Check for incoming serial data
  // Expected format: 3 bytes (servo1_angle, servo2_angle, servo3_angle)
  // Each angle is 0-30 degrees
  
  // REDUCED DEBUG: Track command count to reduce serial output
  // Only print debug messages every 100th command or if corruption detected
  static int command_count = 0;
  bool should_print = false;  // Will be set to true if corruption or every 100th command
  
  // CRITICAL FIX: Aggressively clear any partial data to prevent corruption
  // If we have 1-2 bytes, they're definitely partial - clear them
  int available = Serial.available();
  if (available > 0 && available < 3) {
    if (DEBUG_MODE) {
      Serial.print("WARNING: Clearing partial data (");
      Serial.print(available);
      Serial.println(" bytes)");
    }
    while (Serial.available() > 0) {
      Serial.read(); // Discard partial data
    }
  }
  
  // CRITICAL FIX: Only read if we have exactly 3 bytes (or more, but we'll read exactly 3)
  // This prevents reading partial commands or garbage data
  if (Serial.available() >= 3) {
    // Read 3 bytes atomically for the 3 servos
    int angle1 = Serial.read();
    int angle2 = Serial.read();
    int angle3 = Serial.read();
    
    // CRITICAL FIX: Clear any remaining bytes IMMEDIATELY after reading
    // This prevents leftover data from being interpreted as new commands
    while (Serial.available() > 0) {
      Serial.read(); // Discard any extra bytes
    }
    
    // Calculate checksum for corruption detection
    int received_checksum = angle1 ^ angle2 ^ angle3;
    
    // Increment command count for reduced debug output
    command_count++;
    
    // CRITICAL: Detect suspicious patterns that indicate byte corruption
    // Store original angles for comparison
    int original_angles[3] = {angle1, angle2, angle3};
    bool corruption_detected = false;
    String corruption_reason = "";
    
    // Pattern 1: Any two motors are identical and non-neutral (suggests byte duplication)
    if (angle1 == angle2 && angle1 != NEUTRAL_ANGLE) {
      corruption_detected = true;
      corruption_reason = "M1=M2 (duplication)";
      angle2 = NEUTRAL_ANGLE;  // Force to neutral
    }
    if (angle1 == angle3 && angle1 != NEUTRAL_ANGLE) {
      corruption_detected = true;
      corruption_reason = "M1=M3 (duplication)";
      angle3 = NEUTRAL_ANGLE;  // Force to neutral
    }
    if (angle2 == angle3 && angle2 != NEUTRAL_ANGLE) {
      corruption_detected = true;
      corruption_reason = "M2=M3 (duplication)";
      angle2 = NEUTRAL_ANGLE;
      angle3 = NEUTRAL_ANGLE;  // Force to neutral
    }
    
    // Pattern 2: All motors identical (suggests severe corruption)
    if (angle1 == angle2 && angle2 == angle3 && angle1 != NEUTRAL_ANGLE) {
      corruption_detected = true;
      corruption_reason = "All motors identical (severe corruption)";
      angle1 = NEUTRAL_ANGLE;
      angle2 = NEUTRAL_ANGLE;
      angle3 = NEUTRAL_ANGLE;
    }
    
    // Pattern 3: Sequential pattern suggesting byte shift (e.g., M1=M2, M2=M3)
    if (angle1 == angle2 && angle2 == angle3 && angle1 != NEUTRAL_ANGLE) {
      // Already handled above
    } else if (abs(angle1 - angle2) == 1 && abs(angle2 - angle3) == 1 && 
               angle1 != NEUTRAL_ANGLE && angle2 != NEUTRAL_ANGLE && angle3 != NEUTRAL_ANGLE) {
      // Sequential pattern (e.g., 14,15,16) - suspicious but not necessarily corruption
      // Only flag if all are far from neutral
      if (abs(angle1 - NEUTRAL_ANGLE) > 5 && abs(angle2 - NEUTRAL_ANGLE) > 5 && abs(angle3 - NEUTRAL_ANGLE) > 5) {
        corruption_detected = true;
        corruption_reason = "Sequential pattern detected (possible shift)";
        // Don't force to neutral for this, just log
      }
    }
    
    // REDUCED DEBUG: Only print if corruption detected or every 100th command
    // This reduces serial buffer buildup significantly
    should_print = corruption_detected || (command_count % 100 == 0);
    
    if (DEBUG_MODE && should_print) {
      Serial.print("Received bytes: ");
      Serial.print(original_angles[0]);
      Serial.print(", ");
      Serial.print(original_angles[1]);
      Serial.print(", ");
      Serial.print(original_angles[2]);
      Serial.print(" | Checksum: ");
      Serial.print(received_checksum);
      if (corruption_detected) {
        Serial.print(" | CORRUPTION: ");
        Serial.print(corruption_reason);
      }
      if (command_count % 100 == 0) {
        Serial.print(" | (every 100th)");
      }
      Serial.println();
    }
    
    // If corruption detected, log warning
    if (corruption_detected) {
      if (DEBUG_MODE) {
        Serial.print("CORRUPTION DETECTED: ");
        Serial.print(corruption_reason);
        Serial.print(" | Original: [");
        Serial.print(original_angles[0]);
        Serial.print(", ");
        Serial.print(original_angles[1]);
        Serial.print(", ");
        Serial.print(original_angles[2]);
        Serial.print("] | Corrected: [");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.print(angle2);
        Serial.print(", ");
        Serial.print(angle3);
        Serial.println("]");
      }
    }
    
    // Validate angle range (0-30) - ALL angles must be valid
    if (angle1 >= MIN_ANGLE && angle1 <= MAX_ANGLE &&
        angle2 >= MIN_ANGLE && angle2 <= MAX_ANGLE &&
        angle3 >= MIN_ANGLE && angle3 <= MAX_ANGLE) {
      
      // CRITICAL FIX: Check for motors that should be locked at neutral
      // If a motor receives NEUTRAL_ANGLE for LOCK_THRESHOLD consecutive commands,
      // lock it at neutral to prevent accidental movement
      int angles[3] = {angle1, angle2, angle3};
      for (int i = 0; i < 3; i++) {
        // HARD LOCK CHECK: If motor is hard-locked, FORCE to neutral and ignore command
        if (motorHardLocked[i]) {
          if (angles[i] != NEUTRAL_ANGLE) {
            if (DEBUG_MODE) {
              Serial.print("Motor ");
              Serial.print(i);
              Serial.print(" is HARD-LOCKED - rejecting command ");
              Serial.print(angles[i]);
              Serial.println("°, forcing to neutral");
            }
            angles[i] = NEUTRAL_ANGLE;
          }
          continue;  // Skip soft lock logic for hard-locked motors
        }
        
        if (angles[i] == NEUTRAL_ANGLE) {
          consecutiveNeutralCount[i]++;
          // Lock motor if it's been at neutral for enough consecutive commands
          if (consecutiveNeutralCount[i] >= LOCK_THRESHOLD && !motorLocked[i]) {
            motorLocked[i] = true;
            // If motor has been at neutral for 10+ consecutive commands, hard-lock it
            // This prevents any possibility of movement for motors that should stay still
            if (consecutiveNeutralCount[i] >= 10 && !motorHardLocked[i]) {
              motorHardLocked[i] = true;
              if (DEBUG_MODE) {
                Serial.print("Motor ");
                Serial.print(i);
                Serial.println(" HARD-LOCKED at neutral (received neutral for 10+ consecutive commands)");
              }
            } else if (DEBUG_MODE) {
              Serial.print("Motor ");
              Serial.print(i);
              Serial.println(" LOCKED at neutral (received neutral for 5+ consecutive commands)");
            }
          }
        } else {
          // Motor is trying to move away from neutral
          if (motorLocked[i]) {
            // Motor is locked - check if command is significant enough to unlock
            // Unlock if command is >5 degrees away from neutral (intentional movement)
            int deviation = abs(angles[i] - NEUTRAL_ANGLE);
            if (deviation > 5) {
              // Significant movement detected - unlock motor
              motorLocked[i] = false;
              consecutiveNeutralCount[i] = 0;
              if (DEBUG_MODE) {
                Serial.print("Motor ");
                Serial.print(i);
                Serial.print(" UNLOCKED (received significant command: ");
                Serial.print(angles[i]);
                Serial.println("°)");
              }
            } else {
              // Small movement - keep locked and ignore command
              angles[i] = NEUTRAL_ANGLE;
              if (DEBUG_MODE) {
                Serial.print("Motor ");
                Serial.print(i);
                Serial.println(" is LOCKED - ignoring small command, staying at neutral");
              }
            }
          } else {
            // Motor is not locked - allow movement and reset neutral count
            consecutiveNeutralCount[i] = 0;
          }
        }
      }
      
      // Only update if all angles are valid (after lock checks)
      targetAngles[0] = angles[0];
      targetAngles[1] = angles[1];
      targetAngles[2] = angles[2];
      newCommand = true;
      lastCommandTime = millis(); // Update timestamp
      
      // REDUCED DEBUG: Only print servo updates if corruption or every 100th command
      if (DEBUG_MODE && should_print) {
        Serial.print("Valid angles: Servo1=");
        Serial.print(targetAngles[0]);
        Serial.print("°, Servo2=");
        Serial.print(targetAngles[1]);
        Serial.print("°, Servo3=");
        Serial.print(targetAngles[2]);
        Serial.println("°");
        if (motorLocked[1] || motorLocked[2]) {
          Serial.print("Locked motors: ");
          for (int i = 0; i < 3; i++) {
            if (motorLocked[i]) {
              Serial.print(i);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
      }
    } else {
      // Invalid command - discard and do NOT update servos
      if (DEBUG_MODE) {
        Serial.print("ERROR: Invalid angle range! Angles must be 0-30. Received: ");
        Serial.print(angle1);
        Serial.print(", ");
        Serial.print(angle2);
        Serial.print(", ");
        Serial.println(angle3);
        Serial.println("Command rejected - servos NOT updated");
      }
      // Do NOT set newCommand = true, so servos stay at last valid position
    }
  } else if (Serial.available() > 0 && Serial.available() < 3) {
    // CRITICAL FIX: If we have partial data (1-2 bytes), clear it to prevent
    // it from being combined with future data to form invalid commands
    if (DEBUG_MODE) {
      Serial.print("WARNING: Partial command detected (");
      Serial.print(Serial.available());
      Serial.println(" bytes). Clearing buffer.");
    }
    while (Serial.available() > 0) {
      Serial.read(); // Discard partial data
    }
  }
  
  // CRITICAL FIX: Check for command timeout - if no valid command received
  // for too long, return to neutral position for safety
  if (lastCommandTime > 0 && (millis() - lastCommandTime) > COMMAND_TIMEOUT_MS) {
    if (DEBUG_MODE) {
      Serial.println("WARNING: Command timeout - returning to neutral");
    }
    // Return all servos to neutral
    for (int i = 0; i < 3; i++) {
      targetAngles[i] = NEUTRAL_ANGLE;
    }
    newCommand = true;
    lastCommandTime = 0; // Reset timeout check
  }
  
  // Update servos if new command received
  if (newCommand) {
    for (int i = 0; i < 3; i++) {
      // CRITICAL FIX: If motor is locked (soft or hard), force it to stay at neutral
      // This prevents any accidental movement even if invalid data arrives
      int angleToUse = targetAngles[i];
      if (motorHardLocked[i]) {
        // Hard-locked motors ALWAYS stay at neutral, no exceptions
        angleToUse = NEUTRAL_ANGLE;
        targetAngles[i] = NEUTRAL_ANGLE;
        if (DEBUG_MODE && targetAngles[i] != NEUTRAL_ANGLE) {
          Serial.print("Servo ");
          Serial.print(i);
          Serial.println(" is HARD-LOCKED - forcing to neutral");
        }
      } else if (motorLocked[i] && angleToUse != NEUTRAL_ANGLE) {
        // Soft-locked motors can be unlocked with significant commands, but small ones are ignored
        angleToUse = NEUTRAL_ANGLE;
        targetAngles[i] = NEUTRAL_ANGLE;  // Update stored value too
        if (DEBUG_MODE) {
          Serial.print("Servo ");
          Serial.print(i);
          Serial.println(" is LOCKED - forcing to neutral");
        }
      }
      
      int pwmValue = angleToPWM(angleToUse);
      pwm.setPWM(SERVO_CH[i], 0, pwmValue);
      
      // REDUCED DEBUG: Only print servo PWM updates if corruption or every 100th command
      if (DEBUG_MODE && should_print) {
        Serial.print("Servo ");
        Serial.print(i);
        Serial.print(": angle=");
        Serial.print(angleToUse);
        Serial.print("° -> PWM=");
        Serial.print(pwmValue);
        if (motorHardLocked[i]) {
          Serial.print(" (HARD-LOCKED)");
        } else if (motorLocked[i]) {
          Serial.print(" (LOCKED)");
        }
        Serial.println();
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
