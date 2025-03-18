#include <Arduino.h>
#include <Servo.h>

// PID Controller Class
class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double calculate(double setpoint, double measured_value, double dt) {
        double error = setpoint - measured_value;
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        prev_error_ = error;
        return output;
    }

    void setTunings(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double prev_error_;
    double integral_;
};

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
        // Explicitly set pins as OUTPUT before attaching servos
        pinMode(left_front, OUTPUT);
        pinMode(left_rear, OUTPUT);
        pinMode(right_rear, OUTPUT);
        pinMode(right_front, OUTPUT);
        
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
        
        // Make sure all motors start at CENTER_VALUE (stopped)
        left_font_motor.writeMicroseconds(CENTER_VALUE);
        left_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_font_motor.writeMicroseconds(CENTER_VALUE);
        
        Serial.print("Motors enabled on pins: ");
        Serial.print(left_front);
        Serial.print(", ");
        Serial.print(left_rear);
        Serial.print(", ");
        Serial.print(right_rear);
        Serial.print(", ");
        Serial.println(right_front);
        
        // Small delay after initialization to allow servos to stabilize
        delay(100);
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
        // Use the exact same pattern as in my_move_Code.ino
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
    
    // Added rotation and strafing functions for completeness
    void ccw(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        Serial.print("Rotating CCW with speed: ");
        Serial.println(speed_val);
    }

    void cw(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        Serial.print("Rotating CW with speed: ");
        Serial.println(speed_val);
    }

    void strafe_left(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        Serial.print("Strafing left with speed: ");
        Serial.println(speed_val);
    }

    void strafe_right(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        Serial.print("Strafing right with speed: ");
        Serial.println(speed_val);
    }
};

// Global variables
IRSensor sensor1(A4, "Front");  // Front sensor on A4
IRSensor sensor2(A5, "Back");   // Back sensor on A5
Movement robot;
PIDController pid(0.8, 0.1, 0.05);

// Constants
const float FRONT_SAFETY_DISTANCE = 20.0;  // cm
const float BACK_SAFETY_DISTANCE = 20.0;   // cm
const float TARGET_DISTANCE = 30.0;        // cm
const int MAX_SPEED = 200;
const int MIN_SPEED = 50;
const float DT = 0.05;  // 50ms loop time

unsigned long lastTime = 0;
int currentDirection = 0;  // 0: stop, 1: forward, -1: backward
const int FIXED_SPEED = 250;  // Fixed speed for consistent movement
bool emergencyStop = false;  // Emergency stop flag
bool continuousForward = true;  // Set to true for continuous forward motion

// Convert raw sensor value to distance using linearization
float calculateDistance(int rawValue) {
    // Apply linearization: subtract 61 and divide by 4261.4
    return (rawValue - 61) / 4261.4;
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize the sensors
    sensor1.begin();
    sensor2.begin();
    
    // Set up LED for status indication
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.println("Robot initialized");
    Serial.println("Front sensor on A4, Back sensor on A5");
    Serial.println("Continuous forward mode ENABLED");
    Serial.println("Press 'x' to stop, 'f' to resume forward");
    
    // Give time for hardware to initialize
    delay(1000);
    
    // Start moving forward immediately
    robot.forward(FIXED_SPEED);
}

void loop() {
    // Check for serial commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'x' || cmd == 'X') {
            continuousForward = false;
            robot.stop();
            Serial.println("Movement stopped. Press 'f' to resume forward motion.");
        }
        else if (cmd == 'f' || cmd == 'F') {
            continuousForward = true;
            robot.forward(FIXED_SPEED);
            Serial.println("Continuous forward motion resumed.");
        }
    }
    
    // If continuous forward mode, ensure we're always moving forward
    if (continuousForward) {
        robot.forward(FIXED_SPEED);
    }
    
    // Original sensor code still runs at timed intervals
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= DT * 1000) {
        lastTime = currentTime;
        
        // Only process sensors and movement logic if not in continuous mode
        if (!continuousForward) {
            // Read raw sensor values
            int frontRawValue = sensor1.readRawValue();
            int backRawValue = sensor2.readRawValue();
            
            // Calculate distances with linearization
            float frontDistance = calculateDistance(frontRawValue);
            float backDistance = calculateDistance(backRawValue);
            
            // Print sensor values
            Serial.print(sensor1.getName());
            Serial.print(" Sensor: Raw=");
            Serial.print(frontRawValue);
            Serial.print(", Distance=");
            Serial.print(frontDistance);
            Serial.println(" cm");
            
            Serial.print(sensor2.getName());
            Serial.print(" Sensor: Raw=");
            Serial.print(backRawValue);
            Serial.print(", Distance=");
            Serial.print(backDistance);
            Serial.println(" cm");
            
            // Decision logic for movement
            if (frontDistance < FRONT_SAFETY_DISTANCE) {
                // Front obstacle detected, go backward
                Serial.print("Front obstacle detected at ");
                Serial.print(frontDistance);
                Serial.println(" cm! Moving backward.");
                currentDirection = -1;
            } else if (backDistance < BACK_SAFETY_DISTANCE) {
                // Back obstacle detected, go forward
                Serial.print("Back obstacle detected at ");
                Serial.print(backDistance);
                Serial.println(" cm! Moving forward.");
                currentDirection = 1;
            } else if (frontDistance < TARGET_DISTANCE && backDistance > TARGET_DISTANCE) {
                // Too close to front, too far from back - move backward
                currentDirection = -1;
                Serial.println("Adjusting position - moving backward");
            } else if (frontDistance > TARGET_DISTANCE && backDistance < TARGET_DISTANCE) {
                // Too far from front, too close to back - move forward
                currentDirection = 1;
                Serial.println("Adjusting position - moving forward");
            } else if (abs(frontDistance - TARGET_DISTANCE) < 5.0 && 
                      abs(backDistance - TARGET_DISTANCE) < 5.0) {
                // Both sensors close to target distance
                currentDirection = 0;
                Serial.println("Optimal position reached - stopping");
            }
            
            // Use FIXED_SPEED instead of PID for more consistent movement
            if (currentDirection == 1) {
                robot.forward(FIXED_SPEED);
            } else if (currentDirection == -1) {
                robot.reverse(FIXED_SPEED);
            } else {
                robot.stop();
            }
        }
        
        // LED heartbeat still runs in either mode
        digitalWrite(LED_BUILTIN, (millis() % 1000) < 500);
    }
}