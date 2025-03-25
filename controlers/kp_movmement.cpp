#include <Arduino.h>
#include <Servo.h>

// ==================== PINS ====================
const byte LEFT_FRONT = 46;
const byte LEFT_REAR = 47;
const byte RIGHT_REAR = 50;
const byte RIGHT_FRONT = 51;
const int SERVO_PIN = 7;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// ==================== PARAMETERS ====================
const float TARGET_DISTANCE = 15.0;
const float DISTANCE_TOLERANCE = 1.0;
const int MIN_SPEED = 100;
const int MAX_SPEED = 180;
const float KP = 4.0;
const float KI = 0.1;

// ==================== GLOBAL VARIABLES ====================
Servo leftFrontMotor, leftRearMotor, rightRearMotor, rightFrontMotor, sensorServo;
float errorIntegral = 0;
unsigned long lastControlTime = 0;
double lastDistance = 0;
bool hasReachedTarget = false;

// ==================== ULTRASONIC SENSOR ====================
float getDistance() {
  // Take multiple readings and average them
  const int numReadings = 3;
  float sum = 0;
  int validCount = 0;
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  
  for (int i = 0; i < numReadings; i++) {
    // Send pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure response
    unsigned long t1 = micros();
    unsigned long timeout = t1 + 23200; // Max timeout
    
    // Wait for echo start
    while (digitalRead(ECHO_PIN) == 0) {
      if (micros() > timeout) break;
    }
    
    // If timed out, skip this reading
    if (micros() > timeout) continue;
    
    t1 = micros();
    
    // Wait for echo end
    while (digitalRead(ECHO_PIN) == 1) {
      if (micros() > timeout) break;
    }
    
    // If valid reading, add to sum
    if (micros() <= timeout) {
      float distance = (micros() - t1) / 58.0;
      if (distance > 0 && distance < 400) {
        sum += distance;
        validCount++;
      }
    }
    
    delay(10);
  }
  
  float currentDistance = (validCount > 0) ? (sum / validCount) : -1;
  
  // Print info
  Serial.print("Distance: ");
  Serial.print(currentDistance);
  Serial.println(" cm");
  
  // Update last valid distance if current reading is valid
  if (currentDistance > 0) {
    lastDistance = currentDistance;
  }
  
  return currentDistance;
}

// ==================== MOTOR CONTROL ====================
void setMotorSpeed(int speed) {
  int leftValue = 1500 + speed;
  int rightValue = 1500 - speed;
  
  leftFrontMotor.writeMicroseconds(leftValue);
  leftRearMotor.writeMicroseconds(leftValue);
  rightRearMotor.writeMicroseconds(rightValue);
  rightFrontMotor.writeMicroseconds(rightValue);
}

// ==================== PI CONTROLLER ====================
int calculatePIOutput(float error) {
  // Calculate time since last update
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastControlTime) / 1000.0;
  lastControlTime = currentTime;
  
  // Handle first call or long delays
  if (deltaTime <= 0 || deltaTime > 1.0) deltaTime = 0.1;
  
  // Update integral term with anti-windup
  errorIntegral += error * deltaTime;
  errorIntegral = constrain(errorIntegral, -50, 50);
  
  // Calculate PI terms
  float pTerm = KP * error;
  float iTerm = KI * errorIntegral;
  
  return (int)(pTerm + iTerm);
}

// ==================== MAIN CONTROL FUNCTION ====================
void moveToDistance() {
  // Get current distance
  float currentDistance = getDistance();
  
  // Safety check - if invalid reading, use last known valid distance
  // If no valid reading ever, don't move
  if (currentDistance <= 0) {
    if (lastDistance <= 0) {
      // No valid readings yet, stop and wait
      setMotorSpeed(0);
      Serial.println("WARNING: No valid distance reading. Robot stopped.");
      return;
    }
    // Use last valid reading
    currentDistance = lastDistance;
    Serial.println("Using last valid distance");
  }
  
  // Calculate error
  float error = currentDistance - TARGET_DISTANCE;
  
  // Check if we've reached target
  if (abs(error) <= DISTANCE_TOLERANCE) {
    setMotorSpeed(0);
    hasReachedTarget = true;
    errorIntegral = 0;
    Serial.println("TARGET REACHED");
    return;
  }
  
  // Calculate and apply motor speed
  int speed = calculatePIOutput(error);
  
  // Ensure minimum speed
  if (abs(speed) < MIN_SPEED && speed != 0) {
    speed = (speed > 0) ? MIN_SPEED : -MIN_SPEED;
  }
  
  // Limit maximum speed
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  
  // Apply speed
  setMotorSpeed(speed);
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("Wall Approach Controller");
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize servos
  sensorServo.attach(SERVO_PIN);
  sensorServo.write(90);
  
  leftFrontMotor.attach(LEFT_FRONT);
  leftRearMotor.attach(LEFT_REAR);
  rightRearMotor.attach(RIGHT_REAR);
  rightFrontMotor.attach(RIGHT_FRONT);
  
  // Stop motors initially
  setMotorSpeed(0);
  
  // Initialize controller timing
  lastControlTime = millis();
  
  // Wait for setup to complete
  delay(500);
}

void loop() {
  // Blink LED for status
  digitalWrite(LED_BUILTIN, (millis() % 500) < 250);
  
  // Move to distance if not at target yet
  if (!hasReachedTarget) {
    moveToDistance();
  } 
  // Periodically check position when at target
  else {
    static unsigned long lastCheckTime = 0;
    if (millis() - lastCheckTime > 1000) {
      lastCheckTime = millis();
      
      float currentDistance = getDistance();
      if (currentDistance > 0 && abs(currentDistance - TARGET_DISTANCE) > 1.5) {
        hasReachedTarget = false;
      }
    }
  }
  
  // Small delay to prevent too rapid updates
  delay(50);
}