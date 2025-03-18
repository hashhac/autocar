#include <Arduino.h>
#include <Servo.h>

// Single IR Sensor Class
class IRSensor {
private:
    int sensorPin;
    const char* sensorName;

public:
    // Constructor - initialize with sensor pin
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

// Constants
const int FIXED_SPEED = 250;              // Fixed speed for movement
const float DT = 0.05;                    // 50ms loop time

// Global variables
IRSensor sensor1(A4, "Front");  // Front sensor on A4
IRSensor sensor2(A5, "Back");   // Back sensor on A5
unsigned long lastTime = 0;
int currentDirection = 0;  // 0: stop, 1: forward, -1: backward

// Movement functions
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
}

void cw(int speed) {
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 + speed);
}

void strafe_left(int speed) {
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 - speed);
}

void strafe_right(int speed) {
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 + speed);
}

// Convert raw sensor value to distance - don't use this for decisions
float calculateDistance(int rawValue) {
    return (rawValue - 61) / 4261.4;
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize the sensors
    sensor1.begin();
    sensor2.begin();
    
    // Initialize motors
    enable_motors();
    
    Serial.println("Robot initialized");
    Serial.println("Front sensor on A4, Back sensor on A5");
    Serial.println("Fixed speed movement: 250");
    delay(1000);  // Give time for serial to initialize
}

void loop() {
    unsigned long currentTime = millis();
    
    // Run at fixed time intervals
    if (currentTime - lastTime >= DT * 1000) {
        lastTime = currentTime;
        
        // Read raw sensor values
        int frontRawValue = sensor1.readRawValue();
        int backRawValue = sensor2.readRawValue();
        
        // Calculate distances for display only
        float frontDistance = calculateDistance(frontRawValue);
        float backDistance = calculateDistance(backRawValue);
        
        // Print both sensor values on the same line
        Serial.print("Sensors: Front Raw=");
        Serial.print(frontRawValue);
        Serial.print(" (");
        Serial.print(frontDistance, 4);
        Serial.print(" cm) | Back Raw=");
        Serial.print(backRawValue);
        Serial.print(" (");
        Serial.print(backDistance, 4);
        Serial.print(" cm)");
        
        // WORKING WITH RAW VALUES SINCE THEY ARE MORE RELIABLE:
        
        // Front and back thresholds for RAW values (higher value = closer object)
        const int FRONT_THRESHOLD = 80;  // Adjust based on testing
        const int BACK_THRESHOLD = 80;   // Adjust based on testing
        
        // Decision logic with just 3 conditions using RAW values:
        
        // 1. If front obstacle (raw value > threshold), go backward
        if (frontRawValue > FRONT_THRESHOLD && backRawValue <= BACK_THRESHOLD) {
            Serial.println(" | Backward");
            currentDirection = -1;
        }
        // 2. If back obstacle (raw value > threshold), go forward
        else if (backRawValue > BACK_THRESHOLD && frontRawValue <= FRONT_THRESHOLD) {
            Serial.println(" | Forward");
            currentDirection = 1;
        }
        // 3. If both obstacles or no obstacles, stop
        else if ((frontRawValue > FRONT_THRESHOLD && backRawValue > BACK_THRESHOLD) || 
                (frontRawValue <= FRONT_THRESHOLD && backRawValue <= BACK_THRESHOLD)) {
            Serial.println(" | Stop");
            currentDirection = 0;
        }
        // Default case (shouldn't happen with complete logic above)
        else {
            Serial.println(" | Default: Forward");
            currentDirection = 1;
        }
        
        // Apply movement based on direction with fixed speed
        if (currentDirection == 1) {
            forward(FIXED_SPEED);
        } else if (currentDirection == -1) {
            reverse(FIXED_SPEED);
        } else {
            stop();
        }
    }
}