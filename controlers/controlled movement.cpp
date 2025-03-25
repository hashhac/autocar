#include <Arduino.h>
#include <Servo.h>

// ==================== FUNCTION DECLARATIONS ====================
void moveToDistance(float targetDistance);
float getFilteredDistance();
void setMotorSpeed(int speed);

// ==================== ULTRASONIC SENSOR CLASS ====================
class UltrasonicSensor {
public:
  UltrasonicSensor(Servo& servo, int trigPin, int echoPin)
      : myservo_(servo), trigPin_(trigPin), echoPin_(echoPin), currentAngle_(90) {
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(trigPin_, LOW);
  }

  void initialize() {
    myservo_.write(90);
    currentAngle_ = 90;
    delay(100);
  }

  float getDistance() {
    // Take multiple readings and average them
    const int numReadings = 3;
    float sum = 0;
    int validCount = 0;
    
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(5);
    
    for (int i = 0; i < numReadings; i++) {
      float distance = takeSingleReading();
      if (distance > 0 && distance < 400) {
        sum += distance;
        validCount++;
      }
      delay(10);
    }
    
    return (validCount > 0) ? (sum / validCount) : -1;
  }

  void moveToAngle(int angle) {
    if (angle >= 0 && angle <= 180) {
      myservo_.write(angle);
      currentAngle_ = angle;
      delay(100);
    }
  }

private:
  Servo& myservo_;
  int trigPin_;
  int echoPin_;
  int currentAngle_;
  const unsigned int MAX_DIST = 23200;

  float takeSingleReading() {
    digitalWrite(trigPin_, LOW);
    delayMicroseconds(2);
    
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    unsigned long t1 = micros();
    while (digitalRead(echoPin_) == 0) {
      if (micros() - t1 > MAX_DIST) return -1;
    }

    t1 = micros();
    while (digitalRead(echoPin_) == 1) {
      if (micros() - t1 > MAX_DIST) return -1;
    }

    return (micros() - t1) / 58.0;
  }
};

// ==================== MOTOR CONTROL FUNCTIONS ====================
// Motor pins
const byte LEFT_FRONT = 46;
const byte LEFT_REAR = 47;
const byte RIGHT_REAR = 50;
const byte RIGHT_FRONT = 51;
const int SERVO_PIN = 7;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Servo objects
Servo leftFrontMotor;
Servo leftRearMotor;
Servo rightRearMotor;
Servo rightFrontMotor;
Servo sensorServo;

// Create sensor
UltrasonicSensor sensor(sensorServo, TRIG_PIN, ECHO_PIN);

// State variables
bool hasReachedTarget = false;

// Kalman filter variables
double lastEstimate = 0;
double lastVariance = 100;
const double PROCESS_NOISE = 5;
const double SENSOR_NOISE = 10;

// Controller parameters
const float TARGET_DISTANCE = 15.0;
const float DISTANCE_TOLERANCE = 1.0;
const int MIN_SPEED = 100;
const int MAX_SPEED = 180;
const float DISTANCE_GAIN = 2.5;

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
  setMotorSpeed(0);
}

// Simplified motor control - positive = forward, negative = reverse, 0 = stop
void setMotorSpeed(int speed) {
  // Map speed to servo microseconds (1500 = stop)
  // Positive speed = forward, negative = reverse
  int leftValue = 1500 + speed;
  int rightValue = 1500 - speed;  // Right motors are reversed
  
  leftFrontMotor.writeMicroseconds(leftValue);
  leftRearMotor.writeMicroseconds(leftValue);
  rightRearMotor.writeMicroseconds(rightValue);
  rightFrontMotor.writeMicroseconds(rightValue);
  
  if (speed == 0) {
    Serial.println("Motors stopped");
  } else if (speed > 0) {
    Serial.print("Moving forward with speed: ");
    Serial.println(speed);
  } else {
    Serial.print("Moving backward with speed: ");
    Serial.println(-speed);  // Print positive value
  }
}

// Get distance with Kalman filtering
float getFilteredDistance() {
  sensor.moveToAngle(90);  // Look straight ahead
  float rawDistance = sensor.getDistance();
  
  if (rawDistance <= 0) return lastEstimate;
  
  // Prediction
  double prioriVariance = lastVariance + PROCESS_NOISE;
  
  // Update
  double kalmanGain = prioriVariance / (prioriVariance + SENSOR_NOISE);
  double newEstimate = lastEstimate + kalmanGain * (rawDistance - lastEstimate);
  double newVariance = (1 - kalmanGain) * prioriVariance;
  
  // Store for next time
  lastEstimate = newEstimate;
  lastVariance = newVariance;
  
  Serial.print("Raw dist=");
  Serial.print(rawDistance);
  Serial.print("cm, Filtered=");
  Serial.print(newEstimate);
  Serial.println("cm");
  
  return newEstimate;
}

// Main function to move to a specific distance from wall
void moveToDistance(float targetDistance) {
  float distance = getFilteredDistance();
  float error = distance - targetDistance;
  
  Serial.print("Target=");
  Serial.print(targetDistance);
  Serial.print("cm, Error=");
  Serial.println(error);
  
  // Check if we've reached target
  if (abs(error) <= DISTANCE_TOLERANCE) {
    setMotorSpeed(0);  // Stop
    hasReachedTarget = true;
    Serial.println("TARGET REACHED - Holding position");
    return;
  }
  
  // Calculate motor speed based on error
  // Use adaptive gain based on distance
  float adaptiveGain = DISTANCE_GAIN;
  if (abs(error) < 10) {
    adaptiveGain *= 0.7;  // Lower gain when close
  }
  
  // Calculate speed (positive = forward, negative = reverse)
  int speed = (int)(error * adaptiveGain);
  
  // Ensure minimum effective speed
  if (abs(speed) < MIN_SPEED && speed != 0) {
    speed = (speed > 0) ? MIN_SPEED : -MIN_SPEED;
  }
  
  // Limit maximum speed
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  
  // Apply speed to motors
  setMotorSpeed(speed);
}

// ==================== SETUP & LOOP FUNCTIONS ====================
void setup() {
  Serial.begin(115200);
  
  Serial.println("\n=== WALL APPROACH CONTROLLER ===");
  Serial.println("Target distance: 15 cm");
  
  // Initialize hardware
  sensorServo.attach(SERVO_PIN);
  sensor.initialize();
  setupMotors();
  pinMode(LED_BUILTIN, OUTPUT);
  
  delay(1000);  // Stabilization time
  
  // Initialize Kalman filter with first reading
  float initialReading = sensor.getDistance();
  if (initialReading > 0) {
    lastEstimate = initialReading;
  }
  
  Serial.print("Initial distance: ");
  Serial.print(initialReading);
  Serial.println(" cm");
  
  // If wall is far away, move forward first at constant speed
  if (initialReading > 100) {
    Serial.println("Moving toward wall...");
    setMotorSpeed(150);
    delay(500);
    setMotorSpeed(0);
    delay(100);
  }
}

void loop() {
  // Blink LED to show program is running
  digitalWrite(LED_BUILTIN, (millis() % 500) < 250);
  
  // If not yet at target, keep trying to reach it
  if (!hasReachedTarget) {
    moveToDistance(TARGET_DISTANCE);
  } 
  // If target reached, periodically check and adjust if needed
  else {
    static unsigned long lastCheckTime = 0;
    
    if (millis() - lastCheckTime > 1000) {
      lastCheckTime = millis();
      
      float currentDistance = getFilteredDistance();
      float error = currentDistance - TARGET_DISTANCE;
      
      if (abs(error) > DISTANCE_TOLERANCE * 1.5) {
        Serial.println("Position drift detected - adjusting");
        hasReachedTarget = false;  // Re-enable active control
      }
    }
  }
}