#include <Arduino.h>
#include <Servo.h>

// Single IR Sensor Class
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

// Movement Class - using the working code from my_move_Code.ino
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
    
    // Current speed value
    int speed_val;

public:
    // Constructor
    Movement(int default_speed = 250) {
        speed_val = default_speed;
        
        // Set pins as OUTPUT before attaching servos
        pinMode(left_front, OUTPUT);
        pinMode(left_rear, OUTPUT);
        pinMode(right_rear, OUTPUT);
        pinMode(right_front, OUTPUT);
        
        enable_motors();
    }

    // Enable motors
    void enable_motors() {
        left_font_motor.attach(left_front);
        left_rear_motor.attach(left_rear);
        right_rear_motor.attach(right_rear);
        right_font_motor.attach(right_front);
        
        // Initialize to stopped position
        stop();
    }

    // Disable motors
    void disable_motors() {
        left_font_motor.detach();
        left_rear_motor.detach();
        right_rear_motor.detach();
        right_font_motor.detach();
    }

    // Stop all motors
    void stop() {
        left_font_motor.writeMicroseconds(CENTER_VALUE);
        left_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_font_motor.writeMicroseconds(CENTER_VALUE);
    }

    // Move forward 
    void forward(int custom_speed = 0) {
        int actual_speed = custom_speed > 0 ? custom_speed : speed_val;
        left_font_motor.writeMicroseconds(CENTER_VALUE + actual_speed);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + actual_speed);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - actual_speed);
        right_font_motor.writeMicroseconds(CENTER_VALUE - actual_speed);
    }

    // Move backward
    void reverse(int custom_speed = 0) {
        int actual_speed = custom_speed > 0 ? custom_speed : speed_val;
        left_font_motor.writeMicroseconds(CENTER_VALUE - actual_speed);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - actual_speed);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + actual_speed);
        right_font_motor.writeMicroseconds(CENTER_VALUE + actual_speed);
    }
};

// Global variables
IRSensor sensor1(A4, "Front");  // Front sensor on A4
IRSensor sensor2(A5, "Back");   // Back sensor on A5
Movement robot;

// Constants
const float FRONT_SAFETY_DISTANCE = 20.0;  // cm
const float BACK_SAFETY_DISTANCE = 20.0;   // cm
const int FIXED_SPEED = 250;      // Fixed speed for movement
const float DT = 0.05;            // 50ms loop time

unsigned long lastTime = 0;
bool useSensors = true;           // Flag to control sensor-based movement

// Convert raw sensor value to distance
float calculateDistance(int rawValue) {
    return (rawValue - 61) / 4261.4;
}

void setup() {
    // Initialize serial and sensors
    Serial.begin(115200);
    sensor1.begin();
    sensor2.begin();
    
    Serial.println("Robot initialized");
    Serial.println("Front sensor on A4, Back sensor on A5");
    
    // Allow time for initialization
    delay(500);
}

void loop() {
    // Check for serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'f') {
            useSensors = false;
            Serial.println("Continuous forward motion");
        } else if (cmd == 's') {
            useSensors = true;
            Serial.println("Sensor-guided motion");
        }
    }

    // If not using sensors, just move forward continuously
    if (!useSensors) {
        robot.forward(FIXED_SPEED);
        return;
    }
    
    // When using sensors, check them at fixed intervals
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= DT * 1000) {
        lastTime = currentTime;
        
        // Read sensor values
        int frontRawValue = sensor1.readRawValue();
        int backRawValue = sensor2.readRawValue();
        
        // Calculate distances
        float frontDistance = calculateDistance(frontRawValue);
        float backDistance = calculateDistance(backRawValue);
        
        // Print sensor values
        Serial.print("Front: ");
        Serial.print(frontDistance);
        Serial.print(" cm, Back: ");
        Serial.print(backDistance);
        Serial.println(" cm");
        
        // Simple decision logic - only 3 conditions as requested
        if (frontDistance < FRONT_SAFETY_DISTANCE) {
            // Front obstacle detected, go backward
            Serial.println("Front obstacle - moving backward");
            robot.reverse(FIXED_SPEED);
        } 
        else if (backDistance < BACK_SAFETY_DISTANCE) {
            // Back obstacle detected, go forward
            Serial.println("Back obstacle - moving forward");
            robot.forward(FIXED_SPEED);
        } 
        else {
            // No obstacles, stop
            Serial.println("No obstacles - stopping");
            robot.stop();
        }
    }
}