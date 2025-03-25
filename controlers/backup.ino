#include <Arduino.h>
#include <Servo.h>

// ==================== ULTRASONIC SENSOR CLASS ====================
class UltrasonicSensorMovement {
public:
  UltrasonicSensorMovement(Servo& servo, int trigPin, int echoPin)
      : myservo_(servo), trigPin_(trigPin), echoPin_(echoPin), currentAngle_(90) {
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(trigPin_, LOW);
  }

  void initialize() {
    myservo_.write(90);
    currentAngle_ = 90;
    delay(100); // Reduced delay
  }

  float getDistance() {
    return HC_SR04_range();
  }

  void moveToAngle(int angle) {
    if (angle >= 0 && angle <= 180) {
      myservo_.write(angle);
      currentAngle_ = angle;
      delay(100); // Reduced delay
    }
  }
  
  int getAngle() {
    return currentAngle_;
  }

private:
  Servo& myservo_;
  int trigPin_;
  int echoPin_;
  int currentAngle_;
  const unsigned int MAX_DIST = 23200;

  float HC_SR04_range() {
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    unsigned long t1 = micros();
    while (digitalRead(echoPin_) == 0 && micros() - t1 < MAX_DIST + 1000);
    if (digitalRead(echoPin_) == 0) return -1;

    t1 = micros();
    while (digitalRead(echoPin_) == 1 && micros() - t1 < MAX_DIST + 1000);
    if (digitalRead(echoPin_) == 1) return -1;

    unsigned long pulse_width = micros() - t1;
    return pulse_width / 58.0;
  }
};

// ==================== MOVEMENT FUNCTIONS ====================
// Motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

// Servo objects
Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;

void enable_motors() {
  // Set pins as OUTPUT before attaching servos
  pinMode(left_front, OUTPUT);
  pinMode(left_rear, OUTPUT);
  pinMode(right_rear, OUTPUT);
  pinMode(right_front, OUTPUT);
  
  left_font_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_rear_motor.attach(right_rear);
  right_font_motor.attach(right_front);
  
  Serial.println("Motors enabled");
}

void disable_motors() {
  left_font_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_font_motor.detach();
  
  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
  
  Serial.println("Motors disabled");
}

void stop() {
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
  
  Serial.println("Motors stopped");
}

void forward(int speed) {
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 - speed);
  
  Serial.print("Moving forward with speed: ");
  Serial.println(speed);
}

void reverse(int speed) {
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 + speed);
  
  Serial.print("Moving backward with speed: ");
  Serial.println(speed);
}

void ccw(int speed) {
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 - speed);
  
  Serial.print("Turning left with speed: ");
  Serial.println(speed);
}

void cw(int speed) {
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 + speed);
  
  Serial.print("Turning right with speed: ");
  Serial.println(speed);
}

// NEW: Controlled movement with steering
void forwardWithSteering(int baseSpeed, int steeringAmount) {
  // Positive steeringAmount = right turn
  // Negative steeringAmount = left turn
  int leftSpeed = baseSpeed + steeringAmount;
  int rightSpeed = baseSpeed - steeringAmount;
  
  // Constrain speeds to safe ranges
  leftSpeed = constrain(leftSpeed, -baseSpeed, baseSpeed);
  rightSpeed = constrain(rightSpeed, -baseSpeed, baseSpeed);
  
  left_font_motor.writeMicroseconds(1500 + leftSpeed);
  left_rear_motor.writeMicroseconds(1500 + leftSpeed);
  right_rear_motor.writeMicroseconds(1500 - rightSpeed);
  right_font_motor.writeMicroseconds(1500 - rightSpeed);
  
  Serial.print("Forward with steering: Base=");
  Serial.print(baseSpeed);
  Serial.print(", Steering=");
  Serial.println(steeringAmount);
}

void strafe_left(int speed) {
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 - speed);
  
  Serial.print("Strafing left with speed: ");
  Serial.println(speed);
}

void strafe_right(int speed) {
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 + speed);
  
  Serial.print("Strafing right with speed: ");
  Serial.println(speed);
}

// ==================== GLOBAL VARIABLES ====================
// Ultrasonic Sensor with Servo
Servo sensorServo;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const int SERVO_PIN = 7;

// Create sensor object
UltrasonicSensorMovement ultrasonicSensor(sensorServo, TRIG_PIN, ECHO_PIN);

// Constants
const int FIXED_SPEED = 250;
const int SCAN_SPEED = 150;  // Reduced speed during scanning
const int TURNING_SPEED = 200; // Reduced speed for more controlled turning
const float OBSTACLE_THRESHOLD = 25.0;
const float EMERGENCY_THRESHOLD = 10.0;
const float WALL_THRESHOLD = 15.0;  // Very close wall threshold
const float PATH_CLEAR_THRESHOLD = 30.0; // Threshold to consider path clear enough to proceed
const unsigned long TURN_INCREMENT_DURATION = 200; // Much shorter turn duration for incremental turns
const unsigned long PROXIMITY_CHECK_INTERVAL = 150; // More frequent checks

// ENHANCED: More scanning angles (9 instead of 5)
const int NUM_ANGLES = 9;
const int SCAN_ANGLES[NUM_ANGLES] = {0, 30, 60, 75, 90, 105, 120, 150, 180};

// Control loop parameters
const float STEERING_GAIN = 1.5;       // Reduced gain for smoother steering
const int MAX_STEERING_CORRECTION = 80; // Maximum steering correction

// NEW: Progressive turning parameters
const int INITIAL_TURN_ANGLE = 15;      // Start with small 15° turn
const int TURN_ANGLE_INCREMENT = 15;    // Increase by 15° if needed
const int MAX_TURN_ANGLE = 90;          // Maximum turn angle

// State variables
int currentScanIndex = 0;
bool leftObstacle = false;
bool centerObstacle = false;
bool rightObstacle = false;
bool extremeLeftObstacle = false;  // Very close obstacle at 0°
bool extremeRightObstacle = false; // Very close obstacle at 180°
bool justTurned = false;  // Flag to indicate we just completed a turn
bool completeScanRequired = true;  // Start with a full scan
bool isTurning = false;
bool turningLeft = false;
bool emergencyStop = false;
bool headingCorrection = false;    // Flag for heading correction mode
unsigned long turnStartTime = 0;
unsigned long lastProximityCheckTime = 0;
int currentTurnAngle = 0;          // Current turn angle (for progressive turning)
int turnAttempts = 0;              // Count turn attempts to increase angle if needed

// Tracking closest object
float minDistance = 999.0;
int closestAngle = 90; // Default to looking forward

// Storing all distance readings
float angleDistances[NUM_ANGLES] = {0};

// Directionality
int targetHeading = 90; // Target heading (90 = straight)
int currentHeading = 90; // Current estimated heading

// Terrain map data
float terrainMap[NUM_ANGLES] = {0}; // Full terrain map with all angles
unsigned long lastMapUpdateTime = 0;
const unsigned long MAP_UPDATE_INTERVAL = 2000; // Update map every 2 seconds

// ==================== ENHANCED TERRAIN MAPPING FUNCTION ====================
void updateMap() {
  Serial.println("\n===== COMPLETE TERRAIN MAP =====");
  
  // Store current angle to return to it after mapping
  int currentAngle = ultrasonicSensor.getAngle();
  
  // Scan all angles
  for (int i = 0; i < NUM_ANGLES; i++) {
    int angle = SCAN_ANGLES[i];
    ultrasonicSensor.moveToAngle(angle);
    
    // Take distance reading
    float distance = ultrasonicSensor.getDistance();
    terrainMap[i] = distance;
    
    if (terrainMap[i] < 0) terrainMap[i] = 100; // Invalid reading, assume clear
    
    Serial.print("Angle ");
    Serial.print(angle);
    Serial.print("°: ");
    Serial.print(terrainMap[i]);
    Serial.println(" cm");
  }
  
  // Return to original angle
  ultrasonicSensor.moveToAngle(currentAngle);
  
  // Print enhanced ASCII map representation
  Serial.println("\nTERRAIN PROFILE:");
  Serial.println("----------------------------------------------------------");
  
  // Print angle labels with consistent width
  Serial.print("   ");
  for (int i = 0; i < NUM_ANGLES; i++) {
    int angle = SCAN_ANGLES[i];
    Serial.print(" ");
    // Format the angle to align with the bar graph below
    if (angle < 10) Serial.print("  ");
    else if (angle < 100) Serial.print(" ");
    Serial.print(angle);
    Serial.print("° ");
  }
  Serial.println();
  
  // Print the top line of the bar graph
  Serial.print("  |");
  for (int i = 0; i < NUM_ANGLES; i++) {
    Serial.print("-----|");
  }
  Serial.println();
  
  // Print distance bars (vertical bars showing distance at each angle)
  for (int row = 10; row >= 0; row--) {
    // Print row number (scaled to represent distance)
    int rowDistance = row * 10;
    if (rowDistance < 100) Serial.print(" ");
    Serial.print(rowDistance);
    Serial.print("|");
    
    // Print bars for each angle
    for (int i = 0; i < NUM_ANGLES; i++) {
      float distance = terrainMap[i];
      int barHeight = round(distance / 10);
      
      if (row < barHeight) {
        Serial.print("     |"); // Space (no obstacle)
      } else if (row == barHeight) {
        Serial.print("=====|"); // Top of bar
      } else {
        Serial.print("#####|"); // Obstacle area
      }
    }
    Serial.println();
  }
  
  // Print the bottom line
  Serial.print("  |");
  for (int i = 0; i < NUM_ANGLES; i++) {
    Serial.print("-----|");
  }
  Serial.println();
  
  // IMPROVED: Print numerical values with consistent column width
  Serial.print("Dist|");
  for (int i = 0; i < NUM_ANGLES; i++) {
    int distance = (int)terrainMap[i]; // Convert to integer for cleaner display
    
    // Handle different number lengths for consistent column width
    if (distance < 10) {
      // Single digit (e.g., "7") - Format as " 7  |"
      Serial.print("  ");
      Serial.print(distance);
      Serial.print("  |");
    } else if (distance < 100) {
      // Double digit (e.g., "42") - Format as " 42 |"
      Serial.print(" ");
      Serial.print(distance);
      Serial.print(" |");
    } else if (distance < 1000) {
      // Triple digit (e.g., "100") - Format as "100 |"
      Serial.print(distance);
      Serial.print(" |");
    } else {
      // Values >= 1000 - Truncate to 999 for display
      Serial.print("999 |");
    }
  }
  Serial.println();
  Serial.println("----------------------------------------------------------");
  
  // Update the last map time
  lastMapUpdateTime = millis();
}

// ==================== QUICK FORWARD SCAN FUNCTION ====================
// Checks if it's safe to go forward (center path + slight offset in turning direction)
bool isForwardPathClear() {
  // Check center (90°)
  ultrasonicSensor.moveToAngle(90);
  float centerDistance = ultrasonicSensor.getDistance();
  
  // Check slightly in the direction we're turning (90° ± 15°)
  int offsetAngle = turningLeft ? 105 : 75;
  ultrasonicSensor.moveToAngle(offsetAngle);
  float offsetDistance = ultrasonicSensor.getDistance();
  
  Serial.print("CHECKING PATH: Center=");
  Serial.print(centerDistance);
  Serial.print("cm, Offset(");
  Serial.print(offsetAngle);
  Serial.print("°)=");
  Serial.print(offsetDistance);
  Serial.println("cm");
  
  // Path is clear if both readings are beyond threshold
  bool isClear = (centerDistance > PATH_CLEAR_THRESHOLD && 
                  offsetDistance > PATH_CLEAR_THRESHOLD);
  
  if (isClear) {
    Serial.println("PATH IS CLEAR - CAN PROCEED FORWARD");
  } else {
    Serial.println("PATH STILL BLOCKED - CONTINUE TURNING");
  }
  
  return isClear;
}

// ==================== ANALYZE CLEARANCE FUNCTION ====================
// Analyzes full scan data to find the direction with maximum clearance
void analyzeDirectionalClearance(float &leftClearance, float &centerClearance, float &rightClearance) {
  // Calculate average clearance in each zone, weighted by angle (closer to center = higher weight)
  leftClearance = 0;
  centerClearance = 0;
  rightClearance = 0;
  
  float leftWeight = 0;
  float centerWeight = 0;
  float rightWeight = 0;
  
  // Process all readings with appropriate weighting
  for (int i = 0; i < NUM_ANGLES; i++) {
    int angle = SCAN_ANGLES[i];
    float distance = angleDistances[i];
    
    if (distance <= 0) distance = 100; // Invalid reading treated as clear
    
    // Weight calculation - angles closer to desired direction get higher weight
    float weight = 0;
    
    if (angle < 75) { // Right sector (0-75°)
      weight = 1.0 - (abs(angle - 30) / 45.0); // Peak weight at 30°
      rightClearance += distance * weight;
      rightWeight += weight;
    } else if (angle > 105) { // Left sector (105-180°)
      weight = 1.0 - (abs(angle - 150) / 45.0); // Peak weight at 150°
      leftClearance += distance * weight;
      leftWeight += weight;
    } else { // Center sector (75-105°)
      weight = 1.0 - (abs(angle - 90) / 15.0); // Peak weight at 90°
      centerClearance += distance * weight;
      centerWeight += weight;
    }
  }
  
  // Calculate weighted averages
  if (leftWeight > 0) leftClearance /= leftWeight;
  if (centerWeight > 0) centerClearance /= centerWeight;
  if (rightWeight > 0) rightClearance /= rightWeight;
  
  // Print the clearance analysis
  Serial.println("\nCLEARANCE ANALYSIS:");
  Serial.print("LEFT sector (105-180°): ");
  Serial.print(leftClearance);
  Serial.println(" cm (weighted average)");
  
  Serial.print("CENTER sector (75-105°): ");
  Serial.print(centerClearance);
  Serial.println(" cm (weighted average)");
  
  Serial.print("RIGHT sector (0-75°): ");
  Serial.print(rightClearance);
  Serial.println(" cm (weighted average)");
}

// ==================== PROGRESSIVE TURN FUNCTION ====================
void startProgressiveTurn(bool turnLeft, int initialAngle) {
  // Initialize turning
  isTurning = true;
  turningLeft = turnLeft;
  currentTurnAngle = initialAngle;
  turnStartTime = millis();
  turnAttempts = 1;
  
  // Set heading based on turn direction and angle
  if (turnLeft) {
    currentHeading = targetHeading - currentTurnAngle;
    Serial.print("Starting LEFT turn of ");
  } else {
    currentHeading = targetHeading + currentTurnAngle;
    Serial.print("Starting RIGHT turn of ");
  }
  
  Serial.print(currentTurnAngle);
  Serial.println("° (progressive turning)");
  
  // Start the turn
  if (turnLeft) {
    ccw(TURNING_SPEED);
  } else {
    cw(TURNING_SPEED);
  }
}

// ==================== SCAN ENVIRONMENT FUNCTION ====================
void performFullScan() {
  Serial.println("\nPERFORMING FULL ENVIRONMENT SCAN");
  stop(); // Stop to get accurate readings
  
  // Initialize with safe values
  minDistance = 999.0;
  extremeLeftObstacle = false;
  extremeRightObstacle = false;
  leftObstacle = false;
  centerObstacle = false;
  rightObstacle = false;
  
  // Scan all angles
  for (int i = 0; i < NUM_ANGLES; i++) {
    int angle = SCAN_ANGLES[i];
    ultrasonicSensor.moveToAngle(angle);
    
    // Take distance reading
    float distance = ultrasonicSensor.getDistance();
    angleDistances[i] = distance; // Store all readings
    
    Serial.print("SCAN: Angle ");
    Serial.print(angle);
    Serial.print("°: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Track minimum distance
    if (distance > 0 && distance < minDistance) {
      minDistance = distance;
      closestAngle = angle;
    }
    
    // Check for obstacles
    if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
      // FIXED DIRECTION MAPPING: 
      // Angle 0 = RIGHT
      // Angle 180 = LEFT
      if (angle < 45) {
        rightObstacle = true;
        // Check for extreme close obstacle at far right
        if (angle == 0 && distance < WALL_THRESHOLD) {
          extremeRightObstacle = true;
          Serial.println("WARNING: VERY CLOSE wall on extreme RIGHT!");
        }
      } else if (angle > 135) {
        leftObstacle = true;
        // Check for extreme close obstacle at far left
        if (angle == 180 && distance < WALL_THRESHOLD) {
          extremeLeftObstacle = true;
          Serial.println("WARNING: VERY CLOSE wall on extreme LEFT!");
        }
      } else if (angle >= 75 && angle <= 105) {
        centerObstacle = true;
        Serial.println("Obstacle in CENTER zone");
      } else if (angle >= 45 && angle < 75) {
        rightObstacle = true;
        Serial.println("Obstacle in RIGHT zone");
      } else if (angle > 105 && angle <= 135) {
        leftObstacle = true;
        Serial.println("Obstacle in LEFT zone");
      }
    }
  }
  
  // Full scan complete
  Serial.print("Scan complete - Closest object: ");
  Serial.print(minDistance);
  Serial.print(" cm at angle ");
  Serial.println(closestAngle);
  
  // Return to center position
  ultrasonicSensor.moveToAngle(90);
  completeScanRequired = false;
}

// ==================== REACT FUNCTION ====================
void react() {
  // Update LED heartbeat
  digitalWrite(LED_BUILTIN, (millis() % 1000) < 500);
  
  // If in emergency stop, reverse out then turn
  if (emergencyStop) {
    Serial.println("EMERGENCY MANEUVER");
    
    // Back up slightly
    reverse(FIXED_SPEED);
    delay(300);
    stop();
    
    // Perform a full scan to determine best direction
    performFullScan();
    
    // Calculate clearance in each direction
    float leftClearance, centerClearance, rightClearance;
    analyzeDirectionalClearance(leftClearance, centerClearance, rightClearance);
    
    // Turn toward the direction with most clearance
    if (leftClearance > rightClearance && leftClearance > OBSTACLE_THRESHOLD) {
      startProgressiveTurn(true, INITIAL_TURN_ANGLE);
    } else if (rightClearance > OBSTACLE_THRESHOLD) {
      startProgressiveTurn(false, INITIAL_TURN_ANGLE);
    } else {
      // Very limited space, try backing up more and turning left (arbitrary choice)
      reverse(FIXED_SPEED);
      delay(500);
      stop();
      startProgressiveTurn(true, INITIAL_TURN_ANGLE * 2); // Larger initial angle
    }
    
    // Clear emergency flag
    emergencyStop = false;
    return;
  }
  
  // Handle progressive turning
  if (isTurning) {
    // Check if current turn increment is complete
    if (millis() - turnStartTime > TURN_INCREMENT_DURATION) {
      stop();
      
      // Check if we can proceed forward now
      if (isForwardPathClear()) {
        // Path is clear, stop turning and proceed
        isTurning = false;
        justTurned = true;
        Serial.print("Turn of ");
        Serial.print(currentTurnAngle);
        Serial.println("° complete - path is now clear");
        
        // Start moving forward
        forward(FIXED_SPEED);
      } else {
        // Path not clear, increase turn angle and continue
        turnAttempts++;
        currentTurnAngle = min(INITIAL_TURN_ANGLE * turnAttempts, MAX_TURN_ANGLE);
        
        // Update heading
        if (turningLeft) {
          currentHeading = targetHeading - currentTurnAngle;
          Serial.print("Increasing LEFT turn to ");
        } else {
          currentHeading = targetHeading + currentTurnAngle;
          Serial.print("Increasing RIGHT turn to ");
        }
        
        Serial.print(currentTurnAngle);
        Serial.println("°");
        
        // If we've reached max turn angle and still blocked, try the other direction
        if (currentTurnAngle >= MAX_TURN_ANGLE) {
          Serial.println("Maximum turn angle reached - trying opposite direction");
          
          // Perform a full scan again
          performFullScan();
          
          // Calculate clearance in each direction
          float leftClearance, centerClearance, rightClearance;
          analyzeDirectionalClearance(leftClearance, centerClearance, rightClearance);
          
          // Try turning in the opposite direction if it has decent clearance
          if (turningLeft && rightClearance > OBSTACLE_THRESHOLD) {
            startProgressiveTurn(false, INITIAL_TURN_ANGLE);
          } else if (!turningLeft && leftClearance > OBSTACLE_THRESHOLD) {
            startProgressiveTurn(true, INITIAL_TURN_ANGLE);
          } else {
            // Both directions blocked, back up and turn around
            reverse(FIXED_SPEED);
            delay(500);
            stop();
            
            // Turn in original direction but with larger angle
            startProgressiveTurn(turningLeft, MAX_TURN_ANGLE);
          }
          return;
        }
        
        // Continue turning
        turnStartTime = millis();
        if (turningLeft) {
          ccw(TURNING_SPEED);
        } else {
          cw(TURNING_SPEED);
        }
      }
      return;
    } else {
      // Continue current turn increment
      if (turningLeft) {
        ccw(TURNING_SPEED);
      } else {
        cw(TURNING_SPEED);
      }
      return;
    }
  }
  
  // If a complete scan is required, do it before any movement
  if (completeScanRequired) {
    performFullScan();
    
    // Calculate clearance in each direction using all scan data
    float leftClearance, centerClearance, rightClearance;
    analyzeDirectionalClearance(leftClearance, centerClearance, rightClearance);
    
    // Decide what to do based on full scan results
    if (leftObstacle || rightObstacle || centerObstacle) {
      // Determine movement based on comprehensive clearance data
      if (centerObstacle) {
        // Center blocked - choose the side with more clearance
        if (leftClearance > rightClearance && leftClearance > OBSTACLE_THRESHOLD) {
          // More clearance on left, turn left
          startProgressiveTurn(true, INITIAL_TURN_ANGLE);
        } else if (rightClearance > OBSTACLE_THRESHOLD) {
          // More clearance on right, turn right
          startProgressiveTurn(false, INITIAL_TURN_ANGLE);
        } else {
          // Limited clearance in all directions, back up
          Serial.println("Limited clearance all directions: backing up");
          reverse(FIXED_SPEED);
          delay(400);
          stop();
          
          // Re-check scan after backing up
          completeScanRequired = true;
        }
      } else if (leftObstacle && !rightObstacle) {
        // Left blocked, right clear
        startProgressiveTurn(false, INITIAL_TURN_ANGLE);
      } else if (rightObstacle && !leftObstacle) {
        // Right blocked, left clear
        startProgressiveTurn(true, INITIAL_TURN_ANGLE);
      } else {
        // Both sides have some obstacles - use clearance values
        if (leftClearance > rightClearance + 10) { // 10cm threshold for preference
          // Significantly more room on left
          startProgressiveTurn(true, INITIAL_TURN_ANGLE);
        } else if (rightClearance > leftClearance + 10) {
          // Significantly more room on right
          startProgressiveTurn(false, INITIAL_TURN_ANGLE);
        } else {
          // Similar clearance - use small steering correction
          int steeringCorrection = calculateSteeringCorrection();
          
          Serial.println("Similar clearance both sides: using proportional steering");
          headingCorrection = true;
          forwardWithSteering(SCAN_SPEED, steeringCorrection);
        }
      }
    } else {
      // No obstacles detected, move forward with target heading
      Serial.println("No obstacles detected: Moving forward");
      currentHeading = targetHeading; // Reset to target heading
      headingCorrection = false;
      forward(FIXED_SPEED);
    }
    return;
  }
  
  // If in heading correction mode, apply steering
  if (headingCorrection) {
    int steeringCorrection = calculateSteeringCorrection();
    forwardWithSteering(FIXED_SPEED, steeringCorrection);
    
    // Gradually converge to target heading
    if (currentHeading < targetHeading) {
      currentHeading++;
    } else if (currentHeading > targetHeading) {
      currentHeading--;
    }
    
    if (currentHeading == targetHeading) {
      headingCorrection = false;
    }
    
    return;
  }
  
  // Periodically check the closest object to avoid collisions
  if (millis() - lastProximityCheckTime > PROXIMITY_CHECK_INTERVAL) {
    lastProximityCheckTime = millis();
    
    // If we detect an imminent collision, perform emergency stop
    if (checkClosestObjectProximity()) {
      emergencyStop = true;
      return;
    }
  }
  
  // Periodically update the terrain map
  if (millis() - lastMapUpdateTime > MAP_UPDATE_INTERVAL) {
    updateMap();
  }
  
  // Regular scanning behavior (unchanged)
  // [Scanning code remains as before]
  
  // Scan at the current angle
  int currentAngle = SCAN_ANGLES[currentScanIndex];
  Serial.print("SCANNING: Angle ");
  Serial.print(currentAngle);
  Serial.println("°");
  
  // Move servo to current angle
  ultrasonicSensor.moveToAngle(currentAngle);
  
  // Take distance reading
  float distance = ultrasonicSensor.getDistance();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Store the reading
  angleDistances[currentScanIndex] = distance;
  
  // Check if this is the closest object yet
  if (distance > 0 && distance < minDistance) {
    minDistance = distance;
    closestAngle = currentAngle;
    Serial.print("NEW CLOSEST OBJECT: ");
    Serial.print(minDistance);
    Serial.print(" cm at angle ");
    Serial.println(closestAngle);
  }
  
  // Check for extreme close obstacles
  if (currentAngle == 0 && distance > 0 && distance < WALL_THRESHOLD) {
    extremeRightObstacle = true;
    Serial.println("WARNING: VERY CLOSE wall on extreme RIGHT!");
  } else if (currentAngle == 180 && distance > 0 && distance < WALL_THRESHOLD) {
    extremeLeftObstacle = true;
    Serial.println("WARNING: VERY CLOSE wall on extreme LEFT!");
  }
  
  // Check if obstacle detected and classify by position
  if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
    Serial.print("OBSTACLE DETECTED at angle ");
    Serial.println(currentAngle);
    
    // Categorize obstacle based on angle - Fixed mapping
    if (currentAngle < 75) {
      rightObstacle = true;
      Serial.println("Obstacle on RIGHT side");
    } else if (currentAngle > 105) {
      leftObstacle = true;
      Serial.println("Obstacle on LEFT side");
    } else {
      centerObstacle = true;
      Serial.println("Obstacle in CENTER");
    }
  }
  
  // Move forward at reduced speed while scanning
  forward(SCAN_SPEED);
  
  // Move to next scan angle
  currentScanIndex = (currentScanIndex + 1) % NUM_ANGLES;
  
  // If we've completed a full scan, decide what to do
  if (currentScanIndex == 0) {
    // Check if we need to adjust course or do a complete scan
    if (leftObstacle || centerObstacle || rightObstacle) {
      // Obstacles detected, need to make a decision
      completeScanRequired = true;
    } else {
      // No obstacles, apply any heading correction
      int steeringCorrection = calculateSteeringCorrection();
      
      if (abs(steeringCorrection) > 10) {
        // Apply steering correction while moving forward
        Serial.print("Applying heading correction: ");
        Serial.println(steeringCorrection);
        forwardWithSteering(FIXED_SPEED, steeringCorrection);
      } else {
        // Continue straight ahead at full speed
        Serial.println("Maintaining straight course");
        forward(FIXED_SPEED);
      }
    }
    
    // Reset obstacle flags but keep extreme obstacles
    leftObstacle = false;
    centerObstacle = false;
    rightObstacle = false;
    
    // Reset min distance for next scan cycle, but keep the closest angle
    minDistance = 999.0;
  }
}

// ==================== HELPER FUNCTIONS ====================
bool checkClosestObjectProximity() {
  // Only perform the check if we have a valid closest angle
  if (closestAngle >= 0) {
    // Move to the angle with the closest object
    ultrasonicSensor.moveToAngle(closestAngle);
    
    // Get the current distance
    float currentDistance = ultrasonicSensor.getDistance();
    
    Serial.print("PROXIMITY CHECK at angle ");
    Serial.print(closestAngle);
    Serial.print("°: ");
    Serial.print(currentDistance);
    Serial.println(" cm");
    
    // If we're dangerously close to something, take evasive action
    if (currentDistance > 0 && currentDistance < EMERGENCY_THRESHOLD) {
      Serial.println("EMERGENCY! Object too close!");
      return true;
    }
  }
  
  return false;
}

int calculateSteeringCorrection() {
  // Calculate error (difference between current and target heading)
  int error = currentHeading - targetHeading;
  
  // Apply proportional control
  int correction = (int)(error * STEERING_GAIN);
  
  // Limit maximum correction
  correction = constrain(correction, -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
  
  return correction;
}

// ==================== SETUP & LOOP FUNCTIONS ====================
void setup() {
  // [Setup code remains unchanged]
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN);
  ultrasonicSensor.initialize();
  
  // Initialize motors
  enable_motors();
  
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("===========================================");
  Serial.println("Robot initialized with enhanced navigation:");
  Serial.println("- Full 9-angle terrain mapping visualization");
  Serial.println("- Progressive turning with path checking");
  Serial.println("- Smaller turn increments (15° at a time)");
  Serial.println("- All angles used in decision making");
  Serial.println("===========================================");
  
  delay(1000);  // Initialization delay
  
  // Initial state
  currentScanIndex = 0;
  leftObstacle = false;
  centerObstacle = false;
  rightObstacle = false;
  extremeLeftObstacle = false;
  extremeRightObstacle = false;
  isTurning = false;
  emergencyStop = false;
  justTurned = false;
  completeScanRequired = true;  // Start with a full scan
  headingCorrection = false;
  currentHeading = targetHeading;
  
  // Initialize map with initial readings
  updateMap();
}

void loop() {
  // Call the react function each iteration
  react();
  
  // Minimal delay to prevent too frequent updates
  delay(0);
}