#include <Arduino.h>
#include <Servo.h>
// ==================== ULTRASONIC SENSOR CLASS ====================
class UltrasonicSensorMovement {
    public:
      UltrasonicSensorMovement(Servo& servo, int trigPin, int echoPin)
          : myservo_(servo), trigPin_(trigPin), echoPin_(echoPin), currentAngle_(90) {
        pinMode(trigPin_, OUTPUT);
        pinMode(echoPin_, INPUT);
        digitalWrite(trigPin_, LOW);
      }
    
      void initialize() {
        myservo_.write(90);
        currentAngle_ = 90;
        delay(100); // Reduced delay
      }
    
      float getDistance() {
        // Take multiple readings and average them for more reliable results
        const int numReadings = 3;
        float validReadings[numReadings];
        int validCount = 0;
        float sum = 0;
        
        // Stabilize before taking measurements
        digitalWrite(trigPin_, LOW);
        delayMicroseconds(2);
        
        // Take multiple readings
        for (int i = 0; i < numReadings; i++) {
          float distance = HC_SR04_range();
          if (distance > 0) {
            validReadings[validCount] = distance;
            sum += distance;
            validCount++;
          }
          delay(10); // Small delay between readings
        }
        
        // Calculate average of valid readings
        if (validCount > 0) {
          return sum / validCount;
        } else {
          return -1; // No valid readings
        }
      }
    
      void moveToAngle(int angle) {
        if (angle >= 0 && angle <= 180) {
          myservo_.write(angle);
          currentAngle_ = angle;
          delay(100); // Reduced delay
        }
      }
      
      int getAngle() {
        return currentAngle_;
      }
    
    private:
      Servo& myservo_;
      int trigPin_;
      int echoPin_;
      int currentAngle_;
      const unsigned int MAX_DIST = 23200;
    
      float HC_SR04_range() {
        // Make sure trigger pin is LOW before starting
        digitalWrite(trigPin_, LOW);
        delayMicroseconds(2);
        
        // Send the trigger pulse
        digitalWrite(trigPin_, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin_, LOW);
    
        // Wait for echo to start with timeout
        unsigned long t1 = micros();
        while (digitalRead(echoPin_) == 0) {
          if (micros() - t1 > MAX_DIST + 1000) return -1;
        }
    
        // Measure pulse width (distance)
        t1 = micros();
        while (digitalRead(echoPin_) == 1) {
          if (micros() - t1 > MAX_DIST + 1000) return -1;
        }
    
        unsigned long pulse_width = micros() - t1;
        return pulse_width / 58.0;
      }
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
IRSensor sensor1(A4, "Front_1");  // Front sensor on A4
IRSensor sensor2(A5, "Back_1");   // Back sensor on A5
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

    //intilise ultrasonic sensor
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const int SERVO_PIN = 7;

UltrasonicSensorMovement ultrasonicSensor(sensorServo, TRIG_PIN, ECHO_PIN);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize the sensors
    sensor1.begin();
    sensor2.begin();


    // Initialize motors
    enable_motors();
    
    Serial.println("Robot initialized");
    Serial.println("Fixed speed movement: 250");
    delay(1000);  // Give time for serial to initialize
}
void Side_strafe_error_tolerance(bool is_left,int speed){
    if(is_left){
        strafe_left(speed);
    } else {
        strafe_right(speed);
    }
void keep_distance_from_wall_strafe(int speed, int tollarence, bool is_left){
    ultrasonicSensor.moveToAngle(90);
    float raw_ref_distance = ultrasonicSensor.getDistance();
    Side_strafe_error_tolerance(is_left, FIXED_SPEED);
    delay(400)
    float error_distance = ultrasonicSensor.getDistance() - raw_ref_distance;
    Serial.print("Error distance: ");
    Serial.println(error_distance);
    if(error_distance < -tollarence){
        //code to drive forward 5cm
        forward(speed);
        delay(500);
        stop();
    }else if(error_distance > tollarence){
        //code to drive backward 5cm
        backward(speed);
        delay(500);
        stop();
    }else {
        //nothing needs to be done
    }
}
void point_to_get_distance_kp(bool is_left){
    if(is_left){
        ultrasonicSensor.moveToAngle(180);
    }else{
        ultrasonicSensor.moveToAngle(0);
    }
}
void check_distance_to_strafe_kp_contrller(float k, bool is_left){
    point_to_get_distance_kp(is_left);
    target = ultrasonicSensor.getDistance();
    while(true){
        point_to_get_distance_kp(is_left);
        current = ultrasonicSensor.getDistance();
        error = target - current;
        gain = k * error;
        if (error<5){
            break;
        }
        keep_distance_from_wall_strafe(gain, 5, is_left);
    }
}

void loop() {
    check_distance_to_strafe_kp_contrller(1, true);

}