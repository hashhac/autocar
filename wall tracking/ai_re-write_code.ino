#include <Arduino.h>
#include <Servo.h>

// ==================== PINS ====================
// Motor pins
const byte LEFT_FRONT = 46;
const byte LEFT_REAR = 47;
const byte RIGHT_REAR = 50;
const byte RIGHT_FRONT = 51;
const int SERVO_PIN = 7;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// ==================== CONSTANTS ====================
const int MAX_SPEED = 250;        // Maximum allowed speed
const float SIDE_TOLERANCE = 5.0; // Tolerance for side distance (cm)
const float VERT_TOLERANCE = 7.0; // Tolerance for vertical distance (cm)
const float K_VALUE = 5.0;        // Proportional control constant

// ==================== GLOBAL OBJECTS ====================
Servo leftFrontMotor;
Servo leftRearMotor;
Servo rightRearMotor;
Servo rightFrontMotor;
Servo sensorServo;

// ==================== FUNCTION DECLARATIONS ====================
float getDistance();
void setSensorAngle(int angle);
void stopMotors();
void strafeLeft(int speed);
void strafeRight(int speed);
void moveForward(int speed);
void moveBackward(int speed);
void checkAndStrafeWall(bool isLeft, float targetDistance);
void checkAndAdjustVertical(float targetDistance);

// ==================== ULTRASONIC SENSOR FUNCTIONS ====================
/**
 * Get distance from ultrasonic sensor
 * @return Distance in centimeters, or -1 if invalid reading
 */
float getDistance() {
  // Take multiple readings and average them
  const int numReadings = 3;
  float sum = 0;
  int validCount = 0;
  
  for (int i = 0; i < numReadings; i++) {
    // Send pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Wait for echo
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    
    // Calculate distance
    if (duration > 0) {
      float distance = duration / 58.0;
      if (distance > 0 && distance < 400) {
        sum += distance;
        validCount++;
      }
    }
    
    delay(10);
  }
  
  return (validCount > 0) ? (sum / validCount) : -1;
}

/**
 * Set sensor angle
 * @param angle - Angle in degrees (0 to 180)
 */
void setSensorAngle(int angle) {
  // Constrain angle to valid range
  angle = constrain(angle, 0, 180);
  
  // Set servo position
  sensorServo.write(angle);
  
  // Allow time for servo to reach position
  delay(150);
}

// ==================== MOTOR CONTROL FUNCTIONS ====================
void setupMotors() {
  pinMode(LEFT_FRONT, OUTPUT);
  pinMode(LEFT_REAR, OUTPUT);
  pinMode(RIGHT_REAR, OUTPUT);
  pinMode(RIGHT_FRONT, OUTPUT);
  
  leftFrontMotor.attach(LEFT_FRONT);
  leftRearMotor.attach(LEFT_REAR);
  rightRearMotor.attach(RIGHT_REAR);
  rightFrontMotor.attach(RIGHT_FRONT);
  
  // Initialize to stopped position
  stopMotors();
}

void stopMotors() {
  leftFrontMotor.writeMicroseconds(1500);
  leftRearMotor.writeMicroseconds(1500);
  rightRearMotor.writeMicroseconds(1500);
  rightFrontMotor.writeMicroseconds(1500);
  
  Serial.println("Motors stopped");
}

void strafeLeft(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  leftFrontMotor.writeMicroseconds(1500 - speed);
  leftRearMotor.writeMicroseconds(1500 + speed);
  rightRearMotor.writeMicroseconds(1500 + speed);
  rightFrontMotor.writeMicroseconds(1500 - speed);
  
  Serial.print("Strafing left with speed: ");
  Serial.println(speed);
}

void strafeRight(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  leftFrontMotor.writeMicroseconds(1500 + speed);
  leftRearMotor.writeMicroseconds(1500 - speed);
  rightRearMotor.writeMicroseconds(1500 - speed);
  rightFrontMotor.writeMicroseconds(1500 + speed);
  
  Serial.print("Strafing right with speed: ");
  Serial.println(speed);
}

void moveForward(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  leftFrontMotor.writeMicroseconds(1500 + speed);
  leftRearMotor.writeMicroseconds(1500 + speed);
  rightRearMotor.writeMicroseconds(1500 - speed);
  rightFrontMotor.writeMicroseconds(1500 - speed);
  
  Serial.print("Moving forward with speed: ");
  Serial.println(speed);
}

void moveBackward(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  leftFrontMotor.writeMicroseconds(1500 - speed);
  leftRearMotor.writeMicroseconds(1500 - speed);
  rightRearMotor.writeMicroseconds(1500 + speed);
  rightFrontMotor.writeMicroseconds(1500 + speed);
  
  Serial.print("Moving backward with speed: ");
  Serial.println(speed);
}

// ==================== WALL TRACKING FUNCTIONS ====================
/**
 * Check and adjust horizontal distance to wall with strafing
 * @param isLeft - True if tracking left wall, false if tracking right wall
 * @param targetDistance - Desired distance to maintain from wall (cm)
 */
void checkAndStrafeWall(bool isLeft, float targetDistance) {
  Serial.println("\n--- HORIZONTAL WALL ADJUSTMENT ---");
  
  // Point sensor to correct side
  setSensorAngle(isLeft ? 180 : 0);
  
  // Get current distance
  float currentDistance = getDistance();
  if (currentDistance < 0) {
    Serial.println("Invalid distance reading - aborting horizontal check");
    return;
  }
  
  // Calculate error and required speed
  float error = targetDistance - currentDistance;
  int speed = (int)(K_VALUE * abs(error));
  
  // Ensure minimum effective speed
  if (speed < 100 && speed > 0) {
    speed = 100;
  }
  
  // Limit maximum speed
  speed = constrain(speed, 0, MAX_SPEED);
  
  // Display info
  Serial.print("Side: ");
  Serial.print(isLeft ? "LEFT" : "RIGHT");
  Serial.print(", Current: ");
  Serial.print(currentDistance);
  Serial.print(" cm, Target: ");
  Serial.print(targetDistance);
  Serial.print(" cm, Error: ");
  Serial.print(error);
  Serial.print(" cm, Speed: ");
  Serial.println(speed);
  
  // Check if we're within tolerance
  if (abs(error) <= SIDE_TOLERANCE) {
    Serial.println("Side distance within tolerance - holding position");
    stopMotors();
    return;
  }
  
  // Apply movement based on error
  if (isLeft) {
    // For left wall
    if (error > 0) {
      // Too close to wall - strafe right
      Serial.println("Too close to LEFT wall - strafing RIGHT");
      strafeRight(speed);
      delay(500);  // Move for half a second
      stopMotors(); // Stop to reassess
    } else {
      // Too far from wall - strafe left
      Serial.println("Too far from LEFT wall - strafing LEFT");
      strafeLeft(speed);
      delay(500);  // Move for half a second
      stopMotors(); // Stop to reassess
    }
  } else {
    // For right wall
    if (error > 0) {
      // Too close to wall - strafe left
      Serial.println("Too close to RIGHT wall - strafing LEFT");
      strafeLeft(speed);
      delay(500);  // Move for half a second
      stopMotors(); // Stop to reassess
    } else {
      // Too far from wall - strafe right
      Serial.println("Too far from RIGHT wall - strafing RIGHT");
      strafeRight(speed);
      delay(500);  // Move for half a second
      stopMotors(); // Stop to reassess
    }
  }
}

/**
 * Check and adjust vertical (forward) distance to wall
 * @param targetDistance - Desired forward distance to maintain from wall (cm)
 */
void checkAndAdjustVertical(float targetDistance) {
  Serial.println("\n--- VERTICAL WALL ADJUSTMENT ---");
  
  // Point sensor forward (90 degrees)
  setSensorAngle(90);
  
  // Get current distance
  float currentDistance = getDistance();
  if (currentDistance < 0) {
    Serial.println("Invalid forward distance reading - aborting vertical check");
    return;
  }
  
  // Calculate error and required speed
  float error = targetDistance - currentDistance;
  int speed = (int)(K_VALUE * abs(error));
  
  // Ensure minimum effective speed
  if (speed < 100 && speed > 0) {
    speed = 100;
  }
  
  // Limit maximum speed
  speed = constrain(speed, 0, MAX_SPEED);
  
  // Display info
  Serial.print("Forward distance - Current: ");
  Serial.print(currentDistance);
  Serial.print(" cm, Target: ");
  Serial.print(targetDistance);
  Serial.print(" cm, Error: ");
  Serial.print(error);
  Serial.print(" cm, Speed: ");
  Serial.println(speed);
  
  // Check if we're within tolerance
  if (abs(error) <= VERT_TOLERANCE) {
    Serial.println("Forward distance within tolerance - holding position");
    stopMotors();
    return;
  }
  
  // Apply movement based on error
  if (error > 0) {
    // Too close to wall - move backward
    Serial.println("Too close to forward wall - moving BACKWARD");
    moveBackward(speed);
    delay(500);  // Move for half a second
    stopMotors(); // Stop to reassess
  } else {
    // Too far from wall - move forward
    Serial.println("Too far from forward wall - moving FORWARD");
    moveForward(speed);
    delay(500);  // Move for half a second
    stopMotors(); // Stop to reassess
  }
}

/**
 * Complete wall tracking cycle with horizontal and vertical checks
 * @param isLeft - True if tracking left wall, false if tracking right wall
 * @param sideTarget - Target distance from side wall (cm)
 * @param forwardTarget - Target distance from forward wall (cm)
 */
void trackWallCompleteCycle(bool isLeft, float sideTarget, float forwardTarget) {
  Serial.println("\n====== STARTING WALL TRACKING CYCLE ======");
  Serial.print("Tracking ");
  Serial.print(isLeft ? "LEFT" : "RIGHT");
  Serial.println(" wall");
  
  // First check vertical position
  checkAndAdjustVertical(forwardTarget);
  delay(500);
  
  // Then check horizontal position
  checkAndStrafeWall(isLeft, sideTarget);
  delay(500);
  
  // Check vertical again to make sure it's still good
  checkAndAdjustVertical(forwardTarget);
  
  Serial.println("====== WALL TRACKING CYCLE COMPLETE ======\n");
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  
  // Wait a moment for serial to connect
  delay(1000);
  
  Serial.println("\n===============================");
  Serial.println("=  WALL TRACKING ROBOT v3.0   =");
  Serial.println("===============================");
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN);
  setSensorAngle(90); // Start by looking forward
  
  // Initialize motors
  setupMotors();
  
  // Brief delay to let everything stabilize
  delay(1000);
  
  Serial.println("Robot ready!");
}

void loop() {
  // Blink LED to show we're starting
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Run one complete tracking cycle
  // Parameters: isLeft, sideTargetDistance, forwardTargetDistance
  trackWallCompleteCycle(true, 20.0, 150.0);  // Track left wall
  
  // Pause between cycles
  delay(1000);
}