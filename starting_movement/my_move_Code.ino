#include <Servo.h>  // Need for Servo pulse output

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

// Speed control
int speed_val = 250;  // Fixed speed
char current_direction = ' ';  // Current movement direction

void setup() {
  // Setup serial communication
  Serial.begin(115200);
  Serial.println("Mecanum Robot Movement Control");
  Serial.println("Commands: w (forward), s (reverse), a (rotate left)");
  Serial.println("d (rotate right), q (strafe left), e (strafe right)");
  Serial.println("x (stop), + (increase speed), - (decrease speed)");
  
  // Set pins as OUTPUT before attaching servos
  pinMode(left_front, OUTPUT);
  pinMode(left_rear, OUTPUT);
  pinMode(right_rear, OUTPUT);
  pinMode(right_front, OUTPUT);
  
  // Initialize motors
  enable_motors();
  
  // Indicator LED
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Stop motors initially
  stop();
}

void loop() {
  // Read serial commands if available
  if (Serial.available() > 0) {
    char val = Serial.read();
    
    // Ignore whitespace characters (space, newline, carriage return, etc.)
    if (val == '\n' || val == '\r' || val == ' ') {
      return;
    }
    
    // Process command
    process_command(val);
  }
  
  // Flash LED to show the program is running
  digitalWrite(LED_BUILTIN, (millis() % 1000) < 500);
}

void process_command(char val) {
  // Save the new direction command (if it's a movement command)
  // This prevents stop() from being called for non-movement commands
  switch(val) {
    case 'w':
    case 'W':
    case 's':
    case 'S':
    case 'a':
    case 'A':
    case 'd':
    case 'D':
    case 'q':
    case 'Q':
    case 'e':
    case 'E':
    case 'x':
    case 'X':
      current_direction = val;
      break;
    case '+':
      speed_val = min(500, speed_val + 50);
      Serial.print("Speed increased to: ");
      Serial.println(speed_val);
      break;
    case '-':
      speed_val = max(100, speed_val - 50);
      Serial.print("Speed decreased to: ");
      Serial.println(speed_val);
      break;
  }
  
  // Execute the command based on current direction
  execute_movement(current_direction);
}

void execute_movement(char dir) {
  switch(dir) {
    case 'w':
    case 'W':
      forward();
      Serial.println("Moving Forward");
      break;
    case 's':
    case 'S':
      reverse();
      Serial.println("Moving Backward");
      break;
    case 'a':
    case 'A':
      ccw();
      Serial.println("Rotating CCW");
      break;
    case 'd':
    case 'D':
      cw();
      Serial.println("Rotating CW");
      break;
    case 'q':
    case 'Q':
      strafe_left();
      Serial.println("Strafing Left");
      break;
    case 'e':
    case 'E':
      strafe_right();
      Serial.println("Strafing Right");
      break;
    case 'x':
    case 'X':
    default:
      stop();
      Serial.println("Stopped");
      break;
  }
}

void enable_motors() {
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
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}