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
    delay(200);
  }

  float getDistance() {
    return HC_SR04_range();
  }

  void moveToAngle(int angle) {
    if (angle >= 0 && angle <= 180) {
      myservo_.write(angle);
      currentAngle_ = angle;
      delay(200); // Give time for servo to move
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
const float OBSTACLE_THRESHOLD = 20.0; // Changed to 20cm
const unsigned long TURN_DURATION = 1000; // Turn for 1 second

// Scanning angles
const int NUM_ANGLES = 5;
const int SCAN_ANGLES[NUM_ANGLES] = {0, 45, 90, 135, 180};

// State variables
int currentScanIndex = 0;
bool leftObstacle = false;   // Obstacle on left side (0-45 degrees)
bool centerObstacle = false; // Obstacle in front (90 degrees)
bool rightObstacle = false;  // Obstacle on right side (135-180 degrees)
bool isTurning = false;
bool turningLeft = false;    // Direction of current turn
unsigned long turnStartTime = 0;

// ==================== REACT FUNCTION ====================
void react() {
  // Update LED heartbeat
  digitalWrite(LED_BUILTIN, (millis() % 1000) < 500);
  
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
  
  Serial.println("Robot initialized with direction-sensitive avoidance");
  Serial.println("- Continuously scanning 5 angles while moving");
  Serial.println("- Turn left for right obstacles, right for left obstacles");
  Serial.println("- Detection threshold: 20cm");
  
  delay(1000);  // Initialization delay
  
  // Initial state
  currentScanIndex = 0;
  leftObstacle = false;
  centerObstacle = false;
  rightObstacle = false;
  isTurning = false;
}

// ==================== LOOP FUNCTION ====================
void loop() {
  // Call the react function each iteration
  react();
  
  // Small delay to prevent too frequent updates
  delay(50);
}