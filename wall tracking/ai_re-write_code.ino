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
const float DISTANCE_TOLERANCE = 5.0;  // Tolerance in cm
const float K_VALUE = 5.0;        // Proportional control constant

// ==================== GLOBAL OBJECTS ====================
Servo leftFrontMotor;
Servo leftRearMotor;
Servo rightRearMotor;
Servo rightFrontMotor;
Servo sensorServo;

// ==================== ULTRASONIC SENSOR CLASS ====================
class UltrasonicSensor {
public:
  UltrasonicSensor(Servo& servo, int trigPin, int echoPin)
      : myservo_(servo), trigPin_(trigPin), echoPin_(echoPin) {
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
  }

  void initialize() {
    myservo_.write(90);
    delay(100);
  }

  float getDistance() {
    // Take multiple readings and average them
    const int numReadings = 3;
    float sum = 0;
    int validCount = 0;
    
    for (int i = 0; i < numReadings; i++) {
      // Send pulse
      digitalWrite(trigPin_, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_, LOW);
      
      // Wait for echo
      unsigned long duration = pulseIn(echoPin_, HIGH, 30000);
      
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

  void moveToAngle(int angle) {
    if (angle >= 0 && angle <= 180) {
      myservo_.write(angle);
      delay(100);
    }
  }

private:
  Servo& myservo_;
  int trigPin_;
  int echoPin_;
};

// Create sensor object
UltrasonicSensor ultrasonicSensor(sensorServo, TRIG_PIN, ECHO_PIN);

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
}

void strafeLeft(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  leftFrontMotor.writeMicroseconds(1500 - speed);
  leftRearMotor.writeMicroseconds(1500 + speed);
  rightRearMotor.writeMicroseconds(1500 + speed);
  rightFrontMotor.writeMicroseconds(1500 - speed);
}

void strafeRight(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  leftFrontMotor.writeMicroseconds(1500 + speed);
  leftRearMotor.writeMicroseconds(1500 - speed);
  rightRearMotor.writeMicroseconds(1500 - speed);
  rightFrontMotor.writeMicroseconds(1500 + speed);
}

// ==================== WALL TRACKING FUNCTIONS ====================
// Point sensor toward wall
void pointToWall(bool isLeft) {
  if (isLeft) {
    ultrasonicSensor.moveToAngle(180); // Left side
  } else {
    ultrasonicSensor.moveToAngle(0);   // Right side
  }
}

// Main wall strafing function
void strafeAlongWall(bool isLeft) {
  // Get reference distance
  pointToWall(isLeft);
  float targetDistance = ultrasonicSensor.getDistance();
  
  if (targetDistance < 0) {
    Serial.println("No valid initial distance reading");
    return;
  }
  
  Serial.print("Target distance: ");
  Serial.print(targetDistance);
  Serial.println(" cm");
  
  // Main control loop
  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 15000; // 15 second timeout
  
  while (millis() - startTime < TIMEOUT) {
    // Measure current distance
    pointToWall(isLeft);
    float currentDistance = ultrasonicSensor.getDistance();
    
    // Skip invalid readings
    if (currentDistance < 0) {
      delay(100);
      continue;
    }
    
    // Calculate error and control speed
    float error = targetDistance - currentDistance;
    int speed = (int)(K_VALUE * abs(error));
    
    // Ensure minimum effective speed
    if (speed < 100 && speed > 0) {
      speed = 100;
    }
    
    // Limit maximum speed
    speed = constrain(speed, 0, MAX_SPEED);
    
    // Display info
    Serial.print("Current: ");
    Serial.print(currentDistance);
    Serial.print(" cm, Error: ");
    Serial.print(error);
    Serial.print(" cm, Speed: ");
    Serial.println(speed);
    
    // Check if we're within tolerance
    if (abs(error) <= DISTANCE_TOLERANCE) {
      Serial.println("Within tolerance - holding position");
      stopMotors();
      delay(500);
      continue;
    }
    
    // Apply movement based on error
    if (isLeft) {
      // For left wall
      if (error > 0) {
        // Too close to wall - strafe right
        Serial.println("Strafing right (away from wall)");
        strafeRight(speed);
      } else {
        // Too far from wall - strafe left
        Serial.println("Strafing left (toward wall)");
        strafeLeft(speed);
      }
    } else {
      // For right wall
      if (error > 0) {
        // Too close to wall - strafe left
        Serial.println("Strafing left (away from wall)");
        strafeLeft(speed);
      } else {
        // Too far from wall - strafe right
        Serial.println("Strafing right (toward wall)");
        strafeRight(speed);
      }
    }
    
    // Short delay for sensor update
    delay(100);
  }
  
  // Stop when done
  stopMotors();
  Serial.println("Wall tracking complete");
}

// ==================== ULTRASONIC SENSOR STANDALONE FUNCTIONS ====================

/**
 * Get distance from ultrasonic sensor
 * @param trigPin - Arduino pin connected to sensor's TRIG pin
 * @param echoPin - Arduino pin connected to sensor's ECHO pin
 * @return Distance in centimeters, or -1 if invalid reading
 */
float getUltrasonicDistance(int trigPin, int echoPin) {
  // Take multiple readings and average them for reliability
  const int numReadings = 3;
  float sum = 0;
  int validCount = 0;
  
  for (int i = 0; i < numReadings; i++) {
    // Ensure trigger is LOW to start
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Send 10μs trigger pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Measure the length of the echo pulse
    unsigned long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
    
    // Convert to distance in cm (speed of sound = 343m/s = 34300cm/s)
    // Echo time is round-trip, so divide by 2
    // 1/58 is approximately 34300/2/1000000
    if (duration > 0) {
      float distance = duration / 58.0;
      if (distance > 0 && distance < 400) { // Valid range check
        sum += distance;
        validCount++;
      }
    }
    
    delay(10); // Short delay between readings
  }
  
  // Return average of valid readings, or -1 if none
  return (validCount > 0) ? (sum / validCount) : -1;
}

/**
 * Set servo angle for the ultrasonic sensor
 * @param servo - Servo object to control
 * @param angle - Angle in degrees (0 to 180)
 * @param waitTime - Optional time to wait for servo to reach position (ms)
 */
void setUltrasonicSensorAngle(Servo &servo, int angle, int waitTime = 100) {
  // Constrain angle to valid range
  angle = constrain(angle, 0, 180);
  
  // Set servo position
  servo.write(angle);
  
  // Allow time for servo to reach position
  if (waitTime > 0) {
    delay(waitTime);
  }
}


// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  
  // Wait a moment for serial to connect
  delay(1000);
  
  Serial.println("Wall Tracking Robot");
  Serial.println("------------------");
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN);
  ultrasonicSensor.initialize();
  
  // Initialize motors
  setupMotors();
  
  Serial.println("Ready!");
}

void loop() {
  // Track left wall (true) or right wall (false)
  strafeAlongWall(true);
  
  // Pause between runs
  stopMotors();
  delay(2000);
  
  // Flash LED to show end of cycle
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
}