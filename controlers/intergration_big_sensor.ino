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
const float OBSTACLE_THRESHOLD = 25.0;
const float EMERGENCY_THRESHOLD = 10.0;
const float WALL_THRESHOLD = 15.0;  // Very close wall threshold
const unsigned long TURN_DURATION = 800;
const unsigned long PROXIMITY_CHECK_INTERVAL = 150; // More frequent checks

// ENHANCED: More scanning angles (9 instead of 5)
const int NUM_ANGLES = 9;
const int SCAN_ANGLES[NUM_ANGLES] = {0, 30, 60, 75, 90, 105, 120, 150, 180};
const int MAP_ANGLES[3] = {0, 90, 180}; // Left, Center, Right angles for mapping

// Control loop parameters
const float STEERING_GAIN = 2.0;       // Proportional control gain
const int MAX_STEERING_CORRECTION = 100; // Maximum steering correction

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

// Tracking closest object
float minDistance = 999.0;
int closestAngle = 90; // Default to looking forward

// Storing all distance readings
float angleDistances[NUM_ANGLES] = {0};

// Directionality
int targetHeading = 90; // Target heading (90 = straight)
int currentHeading = 90; // Current estimated heading

// Terrain map data
float terrainMap[3] = {0, 0, 0}; // Left, Center, Right distances
unsigned long lastMapUpdateTime = 0;
const unsigned long MAP_UPDATE_INTERVAL = 2000; // Update map every 2 seconds

// ==================== MAPPING FUNCTION ====================
void updateMap() {
  Serial.println("UPDATING TERRAIN MAP");
  
  // Store current angle to return to it after mapping
  int currentAngle = ultrasonicSensor.getAngle();
  
  // Scan the three mapping angles
  for (int i = 0; i < 3; i++) {
    ultrasonicSensor.moveToAngle(MAP_ANGLES[i]);
    terrainMap[i] = ultrasonicSensor.getDistance();
    
    if (terrainMap[i] < 0) terrainMap[i] = 100; // Invalid reading, assume clear
    
    Serial.print("Map angle ");
    Serial.print(MAP_ANGLES[i]);
    Serial.print(": ");
    Serial.print(terrainMap[i]);
    Serial.println(" cm");
  }
  
  // Return to original angle
  ultrasonicSensor.moveToAngle(currentAngle);
  
  // Print ASCII map representation
  Serial.println("TERRAIN MAP:");
  Serial.println("---------------------");
  
  // Left sector
  Serial.print("Left: ");
  displayDistanceBar(terrainMap[0]);
  
  // Center sector
  Serial.print("Cntr: ");
  displayDistanceBar(terrainMap[1]);
  
  // Right sector
  Serial.print("Rght: ");
  displayDistanceBar(terrainMap[2]);
  
  Serial.println("---------------------");
  
  // Update the last map time
  lastMapUpdateTime = millis();
}

// Helper function to display distance as a bar graph
void displayDistanceBar(float distance) {
  Serial.print("[");
  
  // Calculate how many bar segments to show (max 10)
  int barLength = 0;
  if (distance > 0) {
    barLength = map(constrain(distance, 0, 100), 0, 100, 0, 10);
  }
  
  // Print the bar
  for (int i = 0; i < 10; i++) {
    if (i < barLength) {
      Serial.print(" ");
    } else {
      Serial.print("#");
    }
  }
  
  Serial.print("] ");
  Serial.print(distance);
  Serial.println(" cm");
}

// ==================== PROXIMITY CHECK FUNCTION ====================
bool checkClosestObjectProximity() {
  // Only perform the check if we have a valid closest angle
  if (closestAngle >= 0) {
    // Move to the angle with the closest object
    ultrasonicSensor.moveToAngle(closestAngle);
    
    // Get the current distance
    float currentDistance = ultrasonicSensor.getDistance();
    
    Serial.print("PROXIMITY CHECK at angle ");
    Serial.print(closestAngle);
    Serial.print(": ");
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

// ==================== SCAN ENVIRONMENT FUNCTION ====================
void performFullScan() {
  Serial.println("PERFORMING FULL ENVIRONMENT SCAN");
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
    Serial.print(": ");
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
  
  // Calculate directional weights for steering based on all readings
  calculateSteeringWeights();
  
  // Return to center position
  ultrasonicSensor.moveToAngle(90);
  completeScanRequired = false;
}

// ==================== NEW: STEERING CONTROL FUNCTIONS ====================
// Calculate steering weights based on sensor readings
void calculateSteeringWeights() {
  // Start with neutral heading
  float leftWeight = 0;
  float rightWeight = 0;
  float centerWeight = 0;
  
  // Process all angle readings
  for (int i = 0; i < NUM_ANGLES; i++) {
    int angle = SCAN_ANGLES[i];
    float distance = angleDistances[i];
    
    // Skip invalid readings
    if (distance <= 0) continue;
    
    // Calculate weight based on proximity (closer = stronger weight)
    float weight = 0;
    if (distance < OBSTACLE_THRESHOLD) {
      // Exponential weighting - obstacles closer than threshold have stronger influence
      weight = (OBSTACLE_THRESHOLD - distance) / OBSTACLE_THRESHOLD;
      weight = weight * weight; // Square for stronger effect
      
      // Categorize by angle zone
      if (angle < 75) { // Left side
        leftWeight += weight;
      } else if (angle > 105) { // Right side
        rightWeight += weight;
      } else { // Center
        centerWeight += weight;
      }
    }
  }
  
  // Print weights for debugging
  Serial.print("Steering weights - Left: ");
  Serial.print(leftWeight);
  Serial.print(", Center: ");
  Serial.print(centerWeight);
  Serial.print(", Right: ");
  Serial.println(rightWeight);
  
  // FIXED LOGIC: Calculate heading adjustment based on weights
  if (centerWeight > 0.5) {
    // Center blocked - decide which way to turn
    if (leftWeight >= rightWeight) {
      // MORE weight on left (more obstacles), turn RIGHT
      currentHeading = targetHeading + 45;
      Serial.println("Center & left blocked: turning RIGHT");
    } else {
      // MORE weight on right (more obstacles), turn LEFT
      currentHeading = targetHeading - 45;
      Serial.println("Center & right blocked: turning LEFT");
    }
  } else if (leftWeight > 0.2 && rightWeight > 0.2) {
    // Both sides have obstacles, maintain current heading
    currentHeading = targetHeading;
    Serial.println("Obstacles on both sides: maintaining heading");
  } else if (leftWeight > 0.2) {
    // Left side has obstacles, adjust heading RIGHT
    currentHeading = targetHeading + (int)(leftWeight * 45);
    Serial.println("Left obstacles: turning RIGHT");
  } else if (rightWeight > 0.2) {
    // Right side has obstacles, adjust heading LEFT
    currentHeading = targetHeading - (int)(rightWeight * 45);
    Serial.println("Right obstacles: turning LEFT");
  } else {
    // No significant obstacles, maintain target heading
    currentHeading = targetHeading;
    Serial.println("No obstacles: maintaining heading");
  }
  
  // Constrain heading to valid range
  currentHeading = constrain(currentHeading, 0, 180);
  
  Serial.print("Calculated heading adjustment: ");
  Serial.println(currentHeading);
}

// Calculate steering correction based on current and target heading
int calculateSteeringCorrection() {
  // Calculate error (difference between current and target heading)
  int error = currentHeading - targetHeading;
  
  // Apply proportional control
  int correction = (int)(error * STEERING_GAIN);
  
  // Limit maximum correction
  correction = constrain(correction, -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
  
  return correction;
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
    
    // Turn away from the closest object
    if (closestAngle <= 90) {
      // Object on left/front-left, turn right
      cw(FIXED_SPEED);
    } else {
      // Object on right/front-right, turn left
      ccw(FIXED_SPEED);
    }
    
    delay(700);
    stop();
    
    // Clear emergency flag and force a full scan
    emergencyStop = false;
    completeScanRequired = true;
    
    return;
  }
  
  // If a complete scan is required, do it before any movement
  if (completeScanRequired) {
    performFullScan();
    
    // Decide what to do based on full scan results
    if (leftObstacle || rightObstacle || centerObstacle) {
      // FIXED DIRECTION MAPPING:
      float rightSpace = (angleDistances[0] + angleDistances[1] + angleDistances[2]) / 3.0; // 0°, 30°, 60°
      float centerSpace = angleDistances[4]; // 90°
      float leftSpace = (angleDistances[6] + angleDistances[7] + angleDistances[8]) / 3.0; // 120°, 150°, 180°
      
      Serial.print("SPACE ANALYSIS - Left (120-180°): ");
      Serial.print(leftSpace);
      Serial.print(" cm, Center: ");
      Serial.print(centerSpace);
      Serial.print(" cm, Right (0-60°): ");
      Serial.print(rightSpace);
      Serial.println(" cm");
      
      if (centerObstacle) {
        // Center blocked - choose the side with more space
        if (leftSpace > rightSpace && leftSpace > OBSTACLE_THRESHOLD) {
          Serial.println("Center blocked: TURNING LEFT (more space)");
          isTurning = true;
          turningLeft = true;
          turnStartTime = millis();
          currentHeading = 45; // Set heading to 45° (left)
          ccw(FIXED_SPEED);
        } else if (rightSpace > OBSTACLE_THRESHOLD) {
          Serial.println("Center blocked: TURNING RIGHT (more space)");
          isTurning = true;
          turningLeft = false;
          turnStartTime = millis();
          currentHeading = 135; // Set heading to 135° (right)
          cw(FIXED_SPEED);
        } else {
          // Both directions limited - back up and turn around
          Serial.println("Limited space all directions: backing up");
          reverse(FIXED_SPEED);
          delay(500);
          isTurning = true;
          turningLeft = true; // Default to left
          turnStartTime = millis();
          ccw(FIXED_SPEED);
        }
      } else if (leftObstacle && !rightObstacle) {
        // Left blocked, right clear
        Serial.println("Left blocked: TURNING RIGHT");
        isTurning = true;
        turningLeft = false;
        turnStartTime = millis();
        currentHeading = 120; // Adjust heading right
        cw(FIXED_SPEED);
      } else if (rightObstacle && !leftObstacle) {
        // Right blocked, left clear
        Serial.println("Right blocked: TURNING LEFT");
        isTurning = true;
        turningLeft = true;
        turnStartTime = millis();
        currentHeading = 60; // Adjust heading left
        ccw(FIXED_SPEED);
      } else {
        // Both sides have some obstacles - use weighted steering
        int steeringCorrection = calculateSteeringCorrection();
        
        if (abs(steeringCorrection) > MAX_STEERING_CORRECTION/2) {
          // Large correction needed - do a proper turn
          if (steeringCorrection > 0) {
            // Turn right
            Serial.println("Complex situation: TURNING RIGHT based on weights");
            isTurning = true;
            turningLeft = false;
            turnStartTime = millis();
            cw(FIXED_SPEED);
          } else {
            // Turn left
            Serial.println("Complex situation: TURNING LEFT based on weights");
            isTurning = true;
            turningLeft = true;
            turnStartTime = millis();
            ccw(FIXED_SPEED);
          }
        } else {
          // Small correction - use proportional steering
          Serial.println("Using proportional steering correction");
          headingCorrection = true;
          forwardWithSteering(FIXED_SPEED, steeringCorrection);
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
  
  // If currently turning, check if turn is complete
  if (isTurning) {
    if (millis() - turnStartTime > TURN_DURATION) {
      Serial.print("Turn complete, new heading: ");
      Serial.println(currentHeading);
      isTurning = false;
      justTurned = true;
      completeScanRequired = true; // Force a complete scan after turning
    } else {
      // Continue turning in the current direction
      if (turningLeft) {
        ccw(FIXED_SPEED);
      } else {
        cw(FIXED_SPEED);
      }
      return; // Skip the rest of the function while turning
    }
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
  
  // Scan at the current angle
  int currentAngle = SCAN_ANGLES[currentScanIndex];
  Serial.print("SCANNING: Angle ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
  
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
    
    // Categorize obstacle based on angle
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
    // Update steering weights based on scan data
    calculateSteeringWeights();
    
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

// ==================== SETUP FUNCTION ====================
void setup() {
  // Initialize serial
  Serial.begin(115200);
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN);
  ultrasonicSensor.initialize();
  
  // Initialize motors
  enable_motors();
  
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println("Robot initialized with enhanced angular control system");
  Serial.println("- 9-angle high-resolution scanning");
  Serial.println("- Proportional steering control for precise turning");
  Serial.println("- Heading correction to maintain straight path");
  Serial.println("- Emergency collision avoidance");
  
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

// ==================== LOOP FUNCTION ====================
void loop() {
  // Call the react function each iteration
  react();
  
  // Minimal delay to prevent too frequent updates
  delay(20);
}