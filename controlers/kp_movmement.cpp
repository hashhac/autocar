#include <Arduino.h>
#include <Servo.h>

// ==================== PINS AND CONSTANTS ====================
// Motor pins
const byte LEFT_FRONT = 46;
const byte LEFT_REAR = 47;
const byte RIGHT_REAR = 50;
const byte RIGHT_FRONT = 51;
const int SERVO_PIN = 7;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Target parameters
const float TARGET_DISTANCE = 15.0;
const float DISTANCE_TOLERANCE = 1.0;

// PID controller constants - increased KP for smoother control
const float KP = 6.0;    // Proportional gain (increased from 4.0)
const float KI = 0.1;    // Integral gain
const float KD = 10.0;   // Derivative gain

// Safety limits - only to prevent extreme values
const int ABS_MAX_SPEED = 250;  // Absolute maximum for safety

// Servo objects
Servo leftFrontMotor;
Servo leftRearMotor;
Servo rightRearMotor;
Servo rightFrontMotor;
Servo sensorServo;

// State variables
bool hasReachedTarget = false;

// Distance tracking variables
float lastValidDistance = 0;
bool hasValidDistanceReading = false;

// PID variables
float errorSum = 0;          // For integral term
float lastError = 0;         // For derivative term
unsigned long lastPidTime = 0;  // For time-based calculations

// ==================== FUNCTION DECLARATIONS ====================
float getDistance();
void setMotorSpeed(int speed);
void moveToDistance(float targetDistance);
int calculatePID(float error);

// ==================== DISTANCE SENSING ====================
float getDistance() {
  // Take multiple readings and average them
  const int numReadings = 5;  // Increased for reliability
  float sum = 0;
  int validCount = 0;
  
  // Center the sensor servo first
  sensorServo.write(90);
  delay(10);  // Brief delay to allow servo to stabilize
  
  // Prepare the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  
  // Take multiple readings
  for (int i = 0; i < numReadings; i++) {
    // Send the ultrasonic pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Wait for the echo to start
    unsigned long startTime = micros();
    while (digitalRead(ECHO_PIN) == 0) {
      if (micros() - startTime > 30000) break;  // Timeout after 30ms
    }
    
    // If timed out waiting for echo start, skip this reading
    if (micros() - startTime > 30000) continue;
    
    // Wait for the echo to end
    startTime = micros();
    while (digitalRead(ECHO_PIN) == 1) {
      if (micros() - startTime > 30000) break;  // Timeout after 30ms
    }
    
    // If timed out waiting for echo end, skip this reading
    if (micros() - startTime > 30000) continue;
    
    // Calculate distance
    float distance = (micros() - startTime) / 58.0;  // Convert to cm
    
    // If it's a reasonable distance, add it to our average
    if (distance > 0 && distance < 400) {
      sum += distance;
      validCount++;
    }
    
    delay(10);  // Brief delay between readings
  }
  
  // Calculate average and print result
  float currentDistance = -1;  // Default to invalid
  if (validCount > 0) {
    currentDistance = sum / validCount;
    lastValidDistance = currentDistance;  // Save for future use
    hasValidDistanceReading = true;       // We now have a valid reading
  }
  
  // Always print the distance reading
  Serial.print("Distance reading: ");
  if (currentDistance > 0) {
    Serial.print(currentDistance);
    Serial.println(" cm (valid)");
  } else {
    Serial.print("INVALID");
    if (hasValidDistanceReading) {
      Serial.print(" (last valid: ");
      Serial.print(lastValidDistance);
      Serial.print(" cm)");
    }
    Serial.println();
  }
  
  return currentDistance;
}

// ==================== MOTOR CONTROL ====================
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

void setMotorSpeed(int speed) {
  // Only constrain to absolute limits for safety
  speed = constrain(speed, -ABS_MAX_SPEED, ABS_MAX_SPEED);
  
  // Map speed to servo microseconds (1500 = stop)
  // Positive speed = forward, negative = reverse
  int leftValue = 1500 + speed;
  int rightValue = 1500 - speed;  // Right motors are reversed
  
  leftFrontMotor.writeMicroseconds(leftValue);
  leftRearMotor.writeMicroseconds(leftValue);
  rightRearMotor.writeMicroseconds(rightValue);
  rightFrontMotor.writeMicroseconds(rightValue);
  
  // Always print motor speed for debugging
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

// ==================== PID CONTROLLER ====================
int calculatePID(float error) {
  // Calculate time since last PID calculation for time-based components
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPidTime) / 1000.0; // Convert to seconds
  
  // Handle first call or long delays
  if (deltaTime <= 0 || deltaTime > 1.0) {
    deltaTime = 0.1; // Use default time step
    errorSum = 0;    // Reset integral term
  }
  
  lastPidTime = currentTime;
  
  // Calculate the integral term with anti-windup
  errorSum += error * deltaTime;
  
  // Apply anti-windup by limiting the integral term
  const float MAX_ERROR_SUM = 50.0;
  errorSum = constrain(errorSum, -MAX_ERROR_SUM, MAX_ERROR_SUM);
  
  // Calculate the derivative term
  float errorRate = (error - lastError) / deltaTime;
  lastError = error;
  
  // Calculate PID components
  float pTerm = KP * error;
  float iTerm = KI * errorSum;
  float dTerm = KD * errorRate;
  
  // Calculate total output
  int output = (int)(pTerm + iTerm + dTerm);
  
  // Print PID components for debugging
  Serial.print("PID: P=");
  Serial.print(pTerm);
  Serial.print(", I=");
  Serial.print(iTerm);
  Serial.print(", D=");
  Serial.print(dTerm);
  Serial.print(", Output=");
  Serial.println(output);
  
  return output;
}

// ==================== MAIN CONTROL FUNCTION ====================
void moveToDistance(float targetDistance) {
  // Get current distance
  float currentDistance = getDistance();
  
  // Handle invalid readings
  if (currentDistance <= 0) {
    // If we've never had a valid reading, stop motors for safety
    if (!hasValidDistanceReading) {
      Serial.println("WARNING: No valid distance readings yet. Motors stopped for safety.");
      setMotorSpeed(0);
      return;
    }
    
    // Otherwise use the last valid reading, but with caution
    Serial.println("Using last valid distance reading with caution");
    currentDistance = lastValidDistance;
  }
  
  // Calculate the error
  float error = currentDistance - targetDistance;
  
  Serial.print("Target=");
  Serial.print(targetDistance);
  Serial.print("cm, Current=");
  Serial.print(currentDistance);
  Serial.print("cm, Error=");
  Serial.println(error);
  
  // Check if we've reached target
  if (abs(error) <= DISTANCE_TOLERANCE) {
    setMotorSpeed(0);  // Stop
    hasReachedTarget = true;
    // Reset PID values when target is reached
    errorSum = 0;
    lastError = 0;
    Serial.println("TARGET REACHED - Holding position");
    return;
  }
  
  // Calculate motor speed directly from PID controller without min/max adjustments
  int speed = calculatePID(error);
  
  // Apply speed to motors - no minimum speed enforced
  setMotorSpeed(speed);
}

// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial connection
  }
  
  Serial.println("\n===================================");
  Serial.println("= IMPROVED PID CONTROLLER 4.0 =");
  Serial.println("===================================");
  Serial.println("Target distance: " + String(TARGET_DISTANCE) + " cm");
  Serial.println("PID parameters: KP=" + String(KP) + ", KI=" + String(KI) + ", KD=" + String(KD));
  Serial.println("No minimum speed - direct PID control");
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize servo and sensors
  sensorServo.attach(SERVO_PIN);
  sensorServo.write(90);  // Center position
  delay(500);  // Let servo reach position
  
  // Initialize motors
  setupMotors();
  
  // Initialize PID timing
  lastPidTime = millis();
  
  // Wait for everything to stabilize
  delay(1000);
  
  // Take initial distance reading
  Serial.println("Taking initial distance reading...");
  float initialDistance = getDistance();
  
  if (initialDistance > 0) {
    Serial.print("Initial distance: ");
    Serial.print(initialDistance);
    Serial.println(" cm");
    
    // If wall is far away, move forward first at constant speed
    if (initialDistance > 100) {
      Serial.println("Wall is far away. Moving forward at constant speed first...");
      setMotorSpeed(150);  // Moderate approach speed
      delay(800);
      setMotorSpeed(0);
      delay(500);  // Wait for things to settle
      
      // Take another reading
      Serial.println("Taking new distance reading after initial movement...");
      float newDistance = getDistance();
      if (newDistance > 0) {
        Serial.print("New distance: ");
        Serial.print(newDistance);
        Serial.println(" cm");
      }
    }
  } else {
    Serial.println("WARNING: Could not get valid initial distance reading.");
    Serial.println("Will attempt to get readings during operation.");
  }
  
  Serial.println("Setup complete. Starting main control loop...");
}

void loop() {
  // Blink LED to show program is running
  digitalWrite(LED_BUILTIN, (millis() % 500) < 250);
  
  // Display status every 3 seconds (reduced frequency)
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 3000) {
    lastStatusTime = millis();
    Serial.println("\n--- STATUS UPDATE ---");
    Serial.print("Has reached target: ");
    Serial.println(hasReachedTarget ? "YES" : "NO");
    Serial.print("Has valid distance reading: ");
    Serial.println(hasValidDistanceReading ? "YES" : "NO");
    if (hasValidDistanceReading) {
      Serial.print("Last valid distance: ");
      Serial.print(lastValidDistance);
      Serial.println(" cm");
    }
    Serial.println("--------------------\n");
  }
  
  // If not yet at target, keep trying to reach it
  if (!hasReachedTarget) {
    moveToDistance(TARGET_DISTANCE);
    delay(30);  // Even faster control loop for smoother motion
  } 
  // If target reached, periodically check and adjust if needed
  else {
    static unsigned long lastCheckTime = 0;
    
    if (millis() - lastCheckTime > 1000) {
      lastCheckTime = millis();
      
      Serial.println("Checking position maintenance...");
      float currentDistance = getDistance();
      
      // Only proceed if we got a valid reading
      if (currentDistance > 0) {
        float error = currentDistance - TARGET_DISTANCE;
        
        Serial.print("Current position error: ");
        Serial.print(error);
        Serial.println(" cm");
        
        if (abs(error) > DISTANCE_TOLERANCE * 1.5) {
          Serial.println("Position drift detected - adjusting");
          hasReachedTarget = false;  // Re-enable active control
        } else {
          Serial.println("Position maintained successfully");
        }
      } else {
        Serial.println("Could not check position - invalid distance reading");
      }
    }
  }
}