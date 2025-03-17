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
    
    Serial.println("Robot initialized");
    Serial.println("Front sensor on A4, Back sensor on A5");
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
        
        // Calculate PID control based on current direction and distance
        double setpoint = TARGET_DISTANCE;
        double measured = (currentDirection == 1) ? frontDistance : backDistance;
        double controlSignal = pid.calculate(setpoint, measured, DT);
        
        // Map PID output to motor speed
        int speed = constrain(abs(controlSignal), MIN_SPEED, MAX_SPEED);
        
        Serial.print("PID Control: Setpoint=");
        Serial.print(setpoint);
        Serial.print(", Measured=");
        Serial.print(measured);
        Serial.print(", Output=");
        Serial.print(controlSignal);
        Serial.print(", Speed=");
        Serial.println(speed);
        
        // Apply movement based on direction
        if (currentDirection == 1) {
            robot.forward(speed);
        } else if (currentDirection == -1) {
            robot.reverse(speed);
        } else {
            robot.stop();
        }
    }
}