//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control
//Default motor control pins
#include <Servo.h>  //Need for Servo pulse output

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
    Servo turret_motor;

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
    }

    // Stop all motors
    void stop() {
        left_font_motor.writeMicroseconds(CENTER_VALUE);
        left_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_rear_motor.writeMicroseconds(CENTER_VALUE);
        right_font_motor.writeMicroseconds(CENTER_VALUE);
    }

    // Move forward with specified speed
    void forward(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
    }

    // Move backward with specified speed
    void reverse(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
    }

    // Rotate counter-clockwise with specified speed
    void ccw(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
    }

    // Rotate clockwise with specified speed
    void cw(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
    }

    // Strafe left with specified speed
    void strafe_left(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE - speed_val);
    }

    // Strafe right with specified speed
    void strafe_right(int speed_val) {
        left_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
        left_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_rear_motor.writeMicroseconds(CENTER_VALUE - speed_val);
        right_font_motor.writeMicroseconds(CENTER_VALUE + speed_val);
    }
};

// Example usage
void setup() {
    // Create a movement object - this will enable the motors
    Movement robot;
    
    // Move forward with speed 200
    robot.forward(200);
    delay(2000);
    
    // Stop
    robot.stop();
    delay(1000);
    
    // Turn clockwise with speed 150
    robot.cw(150);
    delay(1000);
    
    // Stop before end of program
    robot.stop();
    
    // When robot goes out of scope, the destructor will disable motors
}

void loop() {
    // Main program loop
}
