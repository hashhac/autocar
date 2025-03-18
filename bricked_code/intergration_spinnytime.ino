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

// Movement Class
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
    // Constructor - initialize and enable motors
    Movement() {
        enable_motors();
    }

    // Destructor - disable motors
    ~Movement() {
        disable_motors();
    }

    // Enable all motors
    void enable_motors() {
        left_font_motor.attach(left_front);
        left_rear_motor.attach(left_rear);
        right_rear_motor.attach(right_rear);
        right_font_motor.attach(right_front);
        Serial.print("Motors enabled on pins: ");
        Serial.print(left_front);
        Serial.print(", ");
        Serial.print(left_rear);
        Serial.print(", ");
        Serial.print(right_rear);
        Serial.print(", ");
        Serial.println(right_front);
    }

    // Disable all motors
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

    // Stop all motors
    void stop() {
        left_font_motor.writeMicroseconds(CENTER_VALUE);
        left_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_font_motor.writeMicroseconds(CENTER_VALUE);
        Serial.println("Motors stopped");
    }

    // Move forward with specified speed
    void forward(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        Serial.print("Moving forward with speed: ");
        Serial.println(speed_val);
    }

    // Move backward with specified speed
    void reverse(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        Serial.print("Moving backward with speed: ");
        Serial.println(speed_val);
    }
};

// Global variables
IRSensor sensor1(A4, "Front");  // Front sensor on A4
IRSensor sensor2(A5, "Back");   // Back sensor on A5
Movement robot;

// Constants
const float FRONT_SAFETY_DISTANCE = 0.05;  // Adjusted for reciprocal values
const float BACK_SAFETY_DISTANCE = 0.05;   // Adjusted for reciprocal values
const float TARGET_DISTANCE = 0.03;        // Adjusted for reciprocal values
const int FIXED_SPEED = 250;              // Fixed speed for movement
const float DT = 0.05;                    // 50ms loop time

unsigned long lastTime = 0;
int currentDirection = 0;  // 0: stop, 1: forward, -1: backward

// Convert raw sensor value to distance using linearization and take reciprocal
float calculateDistance(int rawValue) {
    // Apply linearization: subtract 61, divide by 4261.4, then take reciprocal
    return (1.0 / ((rawValue - 61) / 4261.4));
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize the sensors
    sensor1.begin();
    sensor2.begin();
    
    Serial.println("Robot initialized");
    Serial.println("Front sensor on A4, Back sensor on A5");
    Serial.println("Using reciprocal distance calculation");
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
        
        // Calculate distances with linearization and reciprocal
        float frontDistance = calculateDistance(frontRawValue);
        float backDistance = calculateDistance(backRawValue);
        
        // Print sensor values
        Serial.print(sensor1.getName());
        Serial.print(" Sensor: Raw=");
        Serial.print(frontRawValue);
        Serial.print(", Distance=");
        Serial.print(frontDistance, 6);  // Print with 6 decimal places for reciprocal values
        Serial.println(" (reciprocal units)");
        
        Serial.print(sensor2.getName());
        Serial.print(" Sensor: Raw=");
        Serial.print(backRawValue);
        Serial.print(", Distance=");
        Serial.print(backDistance, 6);  // Print with 6 decimal places for reciprocal values
        Serial.println(" (reciprocal units)");
        
        // Decision logic for movement
        if (frontDistance > FRONT_SAFETY_DISTANCE) {  // Note: Comparison is reversed with reciprocal values
            // Front obstacle detected, go backward
            Serial.print("Front obstacle detected at ");
            Serial.print(frontDistance, 6);
            Serial.println("! Moving backward.");
            currentDirection = -1;
        } else if (backDistance > BACK_SAFETY_DISTANCE) {  // Note: Comparison is reversed with reciprocal values
            // Back obstacle detected, go forward
            Serial.print("Back obstacle detected at ");
            Serial.print(backDistance, 6);
            Serial.println("! Moving forward.");
            currentDirection = 1;
        } else if (frontDistance > TARGET_DISTANCE && backDistance < TARGET_DISTANCE) {  // Note: Comparisons reversed
            // Too close to front, too far from back - move backward
            currentDirection = -1;
            Serial.println("Adjusting position - moving backward");
        } else if (frontDistance < TARGET_DISTANCE && backDistance > TARGET_DISTANCE) {  // Note: Comparisons reversed
            // Too far from front, too close to back - move forward
            currentDirection = 1;
            Serial.println("Adjusting position - moving forward");
        } else if (abs(frontDistance - TARGET_DISTANCE) < 0.01 && 
                  abs(backDistance - TARGET_DISTANCE) < 0.01) {
            // Both sensors close to target distance
            currentDirection = 0;
            Serial.println("Optimal position reached - stopping");
        }
        
        // Apply movement based on direction with fixed speed
        if (currentDirection == 1) {
            robot.forward(FIXED_SPEED);
        } else if (currentDirection == -1) {
            robot.reverse(FIXED_SPEED);
        } else {
            robot.stop();
        }
    }
}