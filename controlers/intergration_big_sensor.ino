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
const float OBSTACLE_THRESHOLD = 25.0; // Changed to 20cm
const float EMERGENCY_THRESHOLD = 10.0; // Even closer distance for emergency maneuvers
const unsigned long TURN_DURATION = 800; // Shorter turn duration
const unsigned long PROXIMITY_CHECK_INTERVAL = 200; // Check closest object every 200ms

// Scanning angles
const int NUM_ANGLES = 5;
const int SCAN_ANGLES[NUM_ANGLES] = {0, 45, 90, 135, 180};
const int MAP_ANGLES[3] = {0, 90, 180}; // Left, Center, Right angles for mapping

// State variables
int currentScanIndex = 0;
bool leftObstacle = false;
bool centerObstacle = false;
bool rightObstacle = false;
bool isTurning = false;
bool turningLeft = false;
bool emergencyStop = false;
unsigned long turnStartTime = 0;
unsigned long lastProximityCheckTime = 0;

// Tracking closest object
float minDistance = 999.0;
int closestAngle = 90; // Default to looking forward

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
    
    // Clear emergency flag
    emergencyStop = false;
    
    // Reset for new scanning
    minDistance = 999.0;
    
    return;
  }
  
  // If currently turning, check if turn is complete
  if (isTurning) {
    if (millis() - turnStartTime > TURN_DURATION) {
      Serial.println("Turn complete, resuming scanning");
      isTurning = false;
      leftObstacle = false;
      centerObstacle = false;
      rightObstacle = false;
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
  
  // Check if this is the closest object yet
  if (distance > 0 && distance < minDistance) {
    minDistance = distance;
    closestAngle = currentAngle;
    Serial.print("NEW CLOSEST OBJECT: ");
    Serial.print(minDistance);
    Serial.print(" cm at angle ");
    Serial.println(closestAngle);
  }
  
  // Check if obstacle detected and classify by position
  if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
    Serial.print("OBSTACLE DETECTED at angle ");
    Serial.println(currentAngle);
    
    // Categorize obstacle based on angle
    if (currentAngle <= 45) {
      // Left side obstacle
      leftObstacle = true;
      Serial.println("Obstacle on LEFT side");
    } else if (currentAngle >= 135) {
      // Right side obstacle
      rightObstacle = true;
      Serial.println("Obstacle on RIGHT side");
    } else {
      // Center/front obstacle
      centerObstacle = true;
      Serial.println("Obstacle in CENTER");
    }
  }
  
  // Move to next scan angle
  currentScanIndex = (currentScanIndex + 1) % NUM_ANGLES;
  
  // If we've completed a full scan (back to first angle), decide what to do
  if (currentScanIndex == 0) {
    if (leftObstacle || centerObstacle || rightObstacle) {
      // Decide turn direction based on where obstacles are
      if (rightObstacle) {
        // Right side obstacle - turn left
        Serial.println("TURNING LEFT to avoid right obstacle");
        isTurning = true;
        turningLeft = true;
        turnStartTime = millis();
        ccw(FIXED_SPEED);
      } else if (leftObstacle) {
        // Left side obstacle - turn right
        Serial.println("TURNING RIGHT to avoid left obstacle");
        isTurning = true;
        turningLeft = false;
        turnStartTime = millis();
        cw(FIXED_SPEED);
      } else if (centerObstacle) {
        // Only center obstacle - turn left by default
        Serial.println("TURNING LEFT to avoid center obstacle");
        isTurning = true;
        turningLeft = true;
        turnStartTime = millis();
        ccw(FIXED_SPEED);
      }
      
      // Reset obstacle detection flags for next scan
      leftObstacle = false;
      centerObstacle = false;
      rightObstacle = false;
    } else {
      // No obstacles detected, continue forward
      Serial.println("No obstacles detected, moving forward");
      forward(FIXED_SPEED);
    }
    
    // Reset min distance for next scan cycle, but keep the closest angle
    minDistance = 999.0;
  } else {
    // Not completed full scan yet, keep moving forward while scanning
    forward(FIXED_SPEED);
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
  
  Serial.println("Robot initialized with proximity monitoring and terrain mapping");
  Serial.println("- Continuously tracking closest object");
  Serial.println("- Emergency maneuvers if objects get too close (10cm)");
  Serial.println("- Terrain mapping of left, center, and right areas");
  Serial.println("- Normal obstacle avoidance threshold: 20cm");
  
  delay(1000);  // Initialization delay
  
  // Initial state
  currentScanIndex = 0;
  leftObstacle = false;
  centerObstacle = false;
  rightObstacle = false;
  isTurning = false;
  emergencyStop = false;
  
  // Initialize map with initial readings
  updateMap();
}

// ==================== LOOP FUNCTION ====================
void loop() {
  // Call the react function each iteration
  react();
  
  // Minimal delay to prevent too frequent updates
  delay(20); // Further reduced delay for better responsiveness
}