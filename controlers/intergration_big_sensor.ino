#include <Arduino.h>
#include <Servo.h>

// ==================== CASE TEMPLATES ====================
// Define possible robot states/cases
enum RobotCase {
  CASE_NORMAL,             // No obstacles detected
  CASE_FRONT_OBSTACLE,     // Obstacle detected in front
  CASE_BACK_OBSTACLE,      // Obstacle detected in back
  CASE_BOTH_OBSTACLES,     // Obstacles detected both front and back
  CASE_CORNER_DETECTED,    // Corner detected by ultrasonic sweep
  CASE_FOLLOW_WALL,        // Wall following mode
  CASE_EMERGENCY_STOP      // Emergency stop triggered
};

RobotCase currentCase = CASE_NORMAL;
bool debug = true; // Set to true for debug messages

// Function to detect which case applies based on sensor readings
RobotCase detectCase(float frontIRDistance, float backIRDistance, float ultrasonicDistance, int servoAngle) {
  // Thresholds for distance sensors
  const float FRONT_THRESHOLD = 20.0;  // cm
  const float BACK_THRESHOLD = 20.0;   // cm
  const float ULTRASONIC_THRESHOLD = 30.0; // cm
  
  // Check various conditions to determine the case
  if (frontIRDistance < FRONT_THRESHOLD && backIRDistance < BACK_THRESHOLD) {
    return CASE_BOTH_OBSTACLES;
  } else if (frontIRDistance < FRONT_THRESHOLD) {
    return CASE_FRONT_OBSTACLE;
  } else if (backIRDistance < BACK_THRESHOLD) {
    return CASE_BACK_OBSTACLE;
  } else if (ultrasonicDistance < ULTRASONIC_THRESHOLD && (servoAngle > 60 && servoAngle < 120)) {
    // Front ultrasonic sensor detecting obstacle
    return CASE_FRONT_OBSTACLE;
  } else if (ultrasonicDistance < ULTRASONIC_THRESHOLD && servoAngle <= 60) {
    // Left side obstacle - potential for wall following
    return CASE_FOLLOW_WALL;
  } else if (ultrasonicDistance < ULTRASONIC_THRESHOLD && servoAngle >= 120) {
    // Right side obstacle - potential for wall following
    return CASE_FOLLOW_WALL;
  }
  
  return CASE_NORMAL;
}

// Function to react to the current case
void reactToCase(RobotCase theCase, Movement& robot, UltrasonicSensorMovement& ultrasonicSensor) {
  const int FIXED_SPEED = 250;
  
  switch (theCase) {
    case CASE_NORMAL:
      if (debug) Serial.println("Case: Normal - Moving forward");
      robot.forward(FIXED_SPEED);
      break;
      
    case CASE_FRONT_OBSTACLE:
      if (debug) Serial.println("Case: Front Obstacle - Moving backward");
      robot.reverse(FIXED_SPEED);
      // Scan left and right to find open path
      ultrasonicSensor.moveToAngle(30); // Look left
      delay(300);
      float leftDist = ultrasonicSensor.getDistance();
      ultrasonicSensor.moveToAngle(150); // Look right
      delay(300);
      float rightDist = ultrasonicSensor.getDistance();
      
      if (leftDist > rightDist) {
        robot.ccw(FIXED_SPEED); // Turn left if more space
        delay(500);
      } else {
        robot.cw(FIXED_SPEED); // Turn right if more space
        delay(500);
      }
      break;
      
    case CASE_BACK_OBSTACLE:
      if (debug) Serial.println("Case: Back Obstacle - Moving forward");
      robot.forward(FIXED_SPEED);
      break;
      
    case CASE_BOTH_OBSTACLES:
      if (debug) Serial.println("Case: Both Obstacles - Stopping and scanning");
      robot.stop();
      // Sweep to find escape route
      ultrasonicSensor.sweepAndDetectCorner();
      break;
      
    case CASE_CORNER_DETECTED:
      if (debug) Serial.println("Case: Corner Detected - Navigating corner");
      // Turn toward the detected corner opening
      // This uses the angle from the ultrasonic sensor's corner detection
      ultrasonicSensor.moveToAngle(90); // Reset to center
      robot.forward(FIXED_SPEED);
      break;
      
    case CASE_FOLLOW_WALL:
      if (debug) Serial.println("Case: Follow Wall");
      // Wall following logic
      // For this simple version, just strafe along the wall
      if (ultrasonicSensor.getAngle() < 90) {
        // Wall on left
        robot.strafe_right(FIXED_SPEED);
      } else {
        // Wall on right
        robot.strafe_left(FIXED_SPEED);
      }
      break;
      
    case CASE_EMERGENCY_STOP:
      if (debug) Serial.println("Case: EMERGENCY STOP");
      robot.stop();
      break;
  }
}

// ==================== ULTRASONIC SENSOR CLASS ====================
class UltrasonicSensorMovement {
public:
  UltrasonicSensorMovement(Servo& servo, HardwareSerial& serial, int trigPin, int echoPin, int minAngle, int maxAngle, int angleIncrement)
      : myservo_(servo), SerialCom_(serial), trigPin_(trigPin), echoPin_(echoPin), 
        sweepMin_(minAngle), sweepMax_(maxAngle), sweepIncrement_(angleIncrement), 
        arraySize_((sweepMax_ - sweepMin_) / sweepIncrement_ + 1),
        currentAngle_(90) {
    pinMode(trigPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(trigPin_, LOW);
  }

  void initialize() {
    myservo_.write(90);
    currentAngle_ = 90;
    delay(500);
  }

  void sweepAndDetectCorner() {
    float distances[arraySize_];
    sweep(distances);
    detectCorner(distances);
  }

  float getDistance() {
    return HC_SR04_range();
  }

  void moveToAngle(int angle) {
    if (angle >= 0 && angle <= 180) {
      myservo_.write(angle);
      currentAngle_ = angle;
    }
  }
  
  int getAngle() {
    return currentAngle_;
  }

private:
  Servo& myservo_;
  HardwareSerial& SerialCom_;
  int trigPin_;
  int echoPin_;
  int sweepMin_;
  int sweepMax_;
  int sweepIncrement_;
  int arraySize_;
  int currentAngle_;
  const unsigned int MAX_DIST = 23200;

  void sweep(float* dist) {
    int count = 0;
    myservo_.write(sweepMin_);
    currentAngle_ = sweepMin_;
    delay(300);
    
    for (int pos = sweepMin_; pos <= sweepMax_; pos += sweepIncrement_) {
      myservo_.write(pos);
      currentAngle_ = pos;
      SerialCom_.print(pos);
      SerialCom_.print(": ");
      dist[count] = HC_SR04_range();
      SerialCom_.println(dist[count]);
      count++;
      delay(200);
    }
    
    myservo_.write(90);
    currentAngle_ = 90;
    delay(300);
  }

  void detectCorner(float* array) {
    float prev_diff = 0;
    float max_diff = prev_diff;
    int index = 0;

    for (int j = 1; j < arraySize_; j++) {
      if ((array[j] != -1) && (array[j - 1] != -1)) {
        float current_diff = abs(array[j] - array[j - 1]);
        if ((current_diff < 0.4 * array[j]) || (current_diff < 0.4 * array[j - 1])) {
          if (current_diff > max_diff) {
            max_diff = current_diff;
            index = j;
          }
        }
        prev_diff = current_diff;
      }
    }

    float found_angle = sweepMin_ + index * sweepIncrement_ - 0.5 * sweepIncrement_;
    myservo_.write(found_angle);
    currentAngle_ = found_angle;
    SerialCom_.print("Corner detected at angle: ");
    SerialCom_.println(found_angle);
  }

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

// ==================== IR SENSOR CLASS ====================
class IRSensor {
private:
    int sensorPin;
    const char* sensorName;

public:
    // Constructor
    IRSensor(int pin, const char* name) {
        sensorPin = pin;
        sensorName = name;
    }
    
    // Initialize sensor
    void begin() {
        pinMode(sensorPin, INPUT);
    }
    
    // Read raw analog value from sensor
    int readRawValue() {
        return analogRead(sensorPin);
    }
    
    // Get sensor name
    const char* getName() {
        return sensorName;
    }
};

// ==================== MOVEMENT CLASS ====================
class Movement {
private:
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

    // Default center value for servos
    const int CENTER_VALUE = 1500;

public:
    // Constructor
    Movement() {
        // Set pins as OUTPUT before attaching servos
        pinMode(left_front, OUTPUT);
        pinMode(left_rear, OUTPUT);
        pinMode(right_rear, OUTPUT);
        pinMode(right_front, OUTPUT);
        
        enable_motors();
    }

    // Destructor
    ~Movement() {
        disable_motors();
    }

    // Enable motors
    void enable_motors() {
        left_font_motor.attach(left_front);
        left_rear_motor.attach(left_rear);
        right_rear_motor.attach(right_rear);
        right_font_motor.attach(right_front);
        
        // Initialize to stopped position
        left_font_motor.writeMicroseconds(CENTER_VALUE);
        left_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_font_motor.writeMicroseconds(CENTER_VALUE);
        
        // Small delay after initialization to allow servos to stabilize
        delay(100);
    }

    // Disable motors
    void disable_motors() {
        left_font_motor.detach();
        left_rear_motor.detach();
        right_rear_motor.detach();
        right_font_motor.detach();

        pinMode(left_front, INPUT);
        pinMode(left_rear, INPUT);
        pinMode(right_rear, INPUT);
        pinMode(right_front, INPUT);
    }

    // Stop all motors
    void stop() {
        left_font_motor.writeMicroseconds(CENTER_VALUE);
        left_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_font_motor.writeMicroseconds(CENTER_VALUE);
        
        if (debug) Serial.println("Motors stopped");
    }

    // Move forward
    void forward(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        
        if (debug) Serial.print("Moving forward with speed: ");
        if (debug) Serial.println(speed_val);
    }

    // Move backward
    void reverse(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        
        if (debug) Serial.print("Moving backward with speed: ");
        if (debug) Serial.println(speed_val);
    }
    
    // Rotate counter-clockwise
    void ccw(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        
        if (debug) Serial.print("Rotating CCW with speed: ");
        if (debug) Serial.println(speed_val);
    }

    // Rotate clockwise
    void cw(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        
        if (debug) Serial.print("Rotating CW with speed: ");
        if (debug) Serial.println(speed_val);
    }

    // Strafe left
    void strafe_left(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        
        if (debug) Serial.print("Strafing left with speed: ");
        if (debug) Serial.println(speed_val);
    }

    // Strafe right
    void strafe_right(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        
        if (debug) Serial.print("Strafing right with speed: ");
        if (debug) Serial.println(speed_val);
    }
};

// ==================== GLOBAL VARIABLES ====================
// IR Sensors
IRSensor frontIR(A4, "Front IR");
IRSensor backIR(A5, "Back IR");

// Ultrasonic Sensor with Servo
Servo sensorServo;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const int SERVO_PIN = 7;
const int SWEEP_MIN = 30;
const int SWEEP_MAX = 150;
const int SWEEP_INC = 15;

// Create sensor and movement objects
UltrasonicSensorMovement ultrasonicSensor(sensorServo, Serial, TRIG_PIN, ECHO_PIN, SWEEP_MIN, SWEEP_MAX, SWEEP_INC);
Movement robot;

// Constants
const int FIXED_SPEED = 250;
const float DT = 0.05;  // 50ms loop time
unsigned long lastTime = 0;

// Convert raw IR sensor value to distance
float calculateIRDistance(int rawValue) {
    return (rawValue - 61) / 4261.4;
}

// ==================== SETUP FUNCTION ====================
void setup() {
    // Initialize serial
    Serial.begin(115200);
    
    // Initialize IR sensors
    frontIR.begin();
    backIR.begin();
    
    // Initialize servo
    sensorServo.attach(SERVO_PIN);
    ultrasonicSensor.initialize();
    
    // Status LED
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.println("Robot initialized with integrated sensors");
    Serial.println("- Front IR on A4, Back IR on A5");
    Serial.println("- Ultrasonic sensor on pins TRIG=" + String(TRIG_PIN) + ", ECHO=" + String(ECHO_PIN));
    Serial.println("- Servo on pin " + String(SERVO_PIN));
    Serial.println("Press 'x' to emergency stop, 'r' to resume");
    
    delay(1000);  // Give time for initialization
}

// ==================== LOOP FUNCTION ====================
void loop() {
    // Check for serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'x' || cmd == 'X') {
            currentCase = CASE_EMERGENCY_STOP;
            Serial.println("EMERGENCY STOP ACTIVATED");
        } else if (cmd == 'r' || cmd == 'R') {
            currentCase = CASE_NORMAL;
            Serial.println("Robot resumed normal operation");
        } else if (cmd == 'd' || cmd == 'D') {
            debug = !debug;
            Serial.println("Debug mode: " + String(debug ? "ON" : "OFF"));
        }
    }
    
    // Run at fixed time intervals for sensor reading and decision making
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= DT * 1000) {
        lastTime = currentTime;
        
        // Only process if not in emergency stop mode
        if (currentCase != CASE_EMERGENCY_STOP) {
            // Read IR sensors
            int frontIRRaw = frontIR.readRawValue();
            int backIRRaw = backIR.readRawValue();
            
            // Calculate IR distances
            float frontIRDistance = calculateIRDistance(frontIRRaw);
            float backIRDistance = calculateIRDistance(backIRRaw);
            
            // Read ultrasonic sensor
            float ultrasonicDistance = ultrasonicSensor.getDistance();
            int servoAngle = ultrasonicSensor.getAngle();
            
            // Log sensor data
            if (debug) {
                Serial.print("Front IR: ");
                Serial.print(frontIRDistance);
                Serial.print(" cm | Back IR: ");
                Serial.print(backIRDistance);
                Serial.print(" cm | Ultrasonic: ");
                Serial.print(ultrasonicDistance);
                Serial.print(" cm at ");
                Serial.print(servoAngle);
                Serial.println(" degrees");
            }
            
            // Detect which case applies based on sensor readings
            RobotCase newCase = detectCase(frontIRDistance, backIRDistance, ultrasonicDistance, servoAngle);
            
            // If case changed, print info
            if (newCase != currentCase && debug) {
                Serial.print("Case changed: ");
                Serial.print(currentCase);
                Serial.print(" -> ");
                Serial.println(newCase);
            }
            
            currentCase = newCase;
        }
        
        // React to the current case
        reactToCase(currentCase, robot, ultrasonicSensor);
        
        // LED heartbeat
        digitalWrite(LED_BUILTIN, (millis() % 1000) < 500);
    }
}