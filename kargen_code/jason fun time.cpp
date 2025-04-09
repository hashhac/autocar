/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED,
  TEST
};

bool testing = true;

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

// IR Sensor
const int IR_LeftPIN = 4;
const int IR_RightPIN = 5;
const int IR_LeftPIN_Back = 6;
const int IR_RightPIN_Back = 7;

// Gyro Sensor
int gyroPin = A3;
int T = 100;          // Loop time in milliseconds
int sensorValue = 0;  // Variable to store raw sensor value

float gyroSupplyVoltage = 5.0;
float gyroZeroVoltage = 0.0;
float gyroSensitivity = 0.007;  // Sensitivity in V/degree/second 0.00555
float rotationThreshold = 1.5;
float gyroRate = 0.0;
float currentAngle = 0.0;
byte serialRead = 0;

// Sweep
int sweep_increment = 5;  //Factors of 180 degrees
int sweep_min = 45;
int sweep_max = 135;
int sweep_range = sweep_max - sweep_min;
int array_size = (sweep_range / sweep_increment) + 1;

bool wall_right = true;

// Move param
float x_kp = 35;
float y_kp = 20;
float ang_kp = 3;

float y_tolerance, x_tolerance, ang_tolerance;



Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;
Servo myservo;

// int speed_val = 100;
int speed_change;

int speed_val = 250;           // Fixed speed
char current_direction = ' ';  // Current movement direction

const float ROTATION_DELAY_PER_DEGREE = 11.875;
const int SENSOR_OFFSET = 61;
const float SENSOR_SCALE = 4261.4;
const int SENSOR_MIN_THRESHOLD = 110;
const int SENSOR_MAX_THRESHOLD = 500;
const int TIMEOUT_DURATION = 10000;

//Serial Pointer
HardwareSerial* SerialCom;

int pos = 0;
void setup(void) {
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // Servo Pin
  myservo.attach(7);

  // Gyro
  pinMode(gyroPin, INPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000);  //settling time but no really needed
}

void loop(void)  //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:  //Lipo Battery Volage OK
      machine_state = running();
      break;
    case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
    case TEST:
      machine_state = test();
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000);  //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  if (testing) {
    return TEST;
  }
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC - SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif
    turret_motor.write(pos);

    if (pos == 0) {
      pos = 45;
    } else {
      pos = 0;
    }
  }

  return RUNNING;
}

STATE test() {
  float sweep_dist[array_size];
  float initial_dist = -1;
  float avg_dist = 0;
  float wall_dist = -1;
  float driving_until_dist = 10;
  int kp = 8;
  int corner_angle = -1;
  int rotate_angle = 90;
  int delay_time = 500;


  SerialCom->println("Start testing");
  // recalibrateGyro();


  //Ultrasonic Check for initial distance
  //DriveUntil with Cond / or rotate

  // myservo.write(180);
  // delay(delay_time);
  // move(15, 40, 0, 1, 1);

  // delay(2000);
  // myservo.write(0);
  // delay(delay_time);
  // move(15, 40, 0, 1, -1);
  // delay(delay_time);
  // move(15, 40, 0, -1, -1);
  // myservo.write(180);
  // delay(delay_time);
  // move(15, 40, 0, -1, 1);

  // setMoveTolerance(3, 3, 5);
  // myservo.write(180);
  // delay(delay_time);
  // move(12, 40, 0, 1, 1);
  // delay(delay_time);
  // orientate();
  // setMoveTolerance(2, 2, 3);
  // delay(delay_time);
  // move(9, 12, 0, 1, 1);
  // delay(delay_time);
  orientate_LR();
  
  delay(delay_time);


  // // // Rotate 90 or 180
  // myservo.write(90);
  // SerialCom->println("Rotation 2");
  // rotate(rotate_angle);
  // delay(delay_time);


  // // // // Measure with ultrasonic to find the longest wall
  // // // // Measure infront
  // SerialCom->println("Finding forward wall");
  // wall_dist = HC_SR04_range();
  // if (wall_dist <= 150) {
  //   rotate(rotate_angle);
  //   wall_right = false;
  //   // SerialCom->println("Wall on the Left");
  //   // myservo.write(180);
  // } else {
  //   // SerialCom->println("Wall on the Right");
  //   // myservo.write(0);
  // }



  // int servo_left = 1;
  // myservo.write(180);
  // delay(delay_time);
  // wall_dist = HC_SR04_range();
  // if (!(wall_dist < 30)) {
  //   myservo.write(0);
  //   servo_left = -1;
  // }


  // SerialCom->print("Servo_left: ");
  // SerialCom->println(servo_left);

  
  // // // x_kp = x_kp*-1;
  // orientate_LR();
  // recalibrateGyro();
  // move(12, 12, 0, 1, servo_left);  //move forward the first time
  // delay(delay_time);
  
  
  
  // int forward = 1;
  // for (int i = 20; i < 110; i = i + 20) {
  //   // strafe
  //   move(13, i, 0, forward, servo_left);
  //   delay(500);
    
  //   forward = -1*forward;
  //   // forward backwards
  //   move(13, i, 0, forward, servo_left);
  //   delay(500);
  // }

  
  // orientate_LR();

  
  // for (int i = 60; i < 110; i = i + 20) {
  //   // strafe
  //   move(12, i, 0, forward, servo_left);
  //   delay(500);
    
  //   forward = -1*forward;
  //   // forward backwards
  //   move(12, i, 0, forward, servo_left);
  //   delay(500);
  // }


  return STOPPED;
}



//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {  //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif







float med_ir_dist(int ir_pin) {
  int analog_value;
  float distance;

  analog_value = analogRead(ir_pin);
  if ((analog_value > 110) && (analog_value < 550)) {
    distance = (2118.6) / (analog_value - 20.072);
  } else {
    distance = -1;
  }

  return distance;
}

float long_ir_dist(int ir_pin) {
  int analog_value;
  float distance;
  int correction = 2;

  analog_value = analogRead(ir_pin);
  if ((analog_value > 110) && (analog_value < 550)) {
    distance = (4261.4) / (analog_value - 61.06) + correction;
  } else {
    distance = -1;
  }

  return distance;
}


void orientate() {
  float Kp = 3;
  float ir_left, ir_right;
  speed_val = 75;
  int stable_count = 0;
  int required_count = 3;
  float error = 1.0;
  float buffer_left, buffer_right;
  ir_left = 30;
  ir_right = 30;

  do {
    buffer_left = med_ir_dist(IR_LeftPIN);
    buffer_right = med_ir_dist(IR_RightPIN);
    SerialCom->print("Left IR: ");
    SerialCom->println(ir_left);
    SerialCom->print("Right IR: ");
    SerialCom->println(ir_right);


    if (!(buffer_left == -1)) {
      ir_left = buffer_left;
    }
    if (!(buffer_right == -1)) {
      ir_right = buffer_right;
    }



    error = ir_left - ir_right;
    int turn_speed = Kp * error;

    if (abs(error) <= 0.15) {
      stable_count++;  // Increase counter if within tolerance
    } else {
      stable_count = 0;
    }

    speed_val = constrain(abs(turn_speed), 75, 100);

    if (error > 0) {
      cw();
    } else {
      ccw();
    }

    delay(25);  // Small delay for fast response
  } while ((stable_count < required_count) && (!(ir_left == 30)));

  stop();
}

// Orientate Constants (Consider adjusting these based on testing)
const float ORIENTATE_TOLERANCE = 0.5;  // Tolerance from second code (cm)
const int ORIENTATE_MIN_SPEED = 60;   // Minimum speed for rotation
const int ORIENTATE_MAX_SPEED = 100;  // Maximum speed for rotation
const int ORIENTATE_STABLE_COUNT = 4; // Required consecutive readings within tolerance (from second code)
const float ORIENTATE_KP = 3.0;       // Proportional gain (from second code)

void orientate_LR() {
    SerialCom->println("Orientate fn (Align Parallel using IR)...");

    // Use constants/values adapted from both codes
    float tolerance = ORIENTATE_TOLERANCE;
    int required_count = ORIENTATE_STABLE_COUNT;
    float Kp = ORIENTATE_KP;

    // Variables for sensor readings and control
    float ir_left = -1.0;  // Initialize to invalid state
    float ir_right = -1.0; // Initialize to invalid state
    float error = 0.0;
    int stable_count = 0;
    int rotation_speed = 0;
    float buffer_left, buffer_right; // Temporary storage for sensor readings

    unsigned long lastPrintTime = 0; // Timer for periodic printing

    do {
        // Read sensors into temporary buffers
        buffer_left = med_ir_dist(IR_LeftPIN);
        buffer_right = med_ir_dist(IR_RightPIN);

        // Update main variables only if readings are valid ( > 0 )
        // This uses the "last known good value" approach similar to second code
        if (buffer_left > 0) {
            ir_left = buffer_left;
        }
        if (buffer_right > 0) {
            ir_right = buffer_right;
        }

        // Print sensor readings periodically
        if (millis() - lastPrintTime > 100) { // Print every 100ms
            SerialCom->print("  Orientate -> L:");
            SerialCom->print(ir_left > 0 ? String(ir_left, 1) : "Err"); // Show last good or Err
            SerialCom->print(" | R:");
            SerialCom->print(ir_right > 0 ? String(ir_right, 1) : "Err"); // Show last good or Err
            lastPrintTime = millis(); // Reset print timer

            // Only calculate and print error etc. if we have valid readings for both
            if (ir_left > 0 && ir_right > 0) {
                 error = ir_left - ir_right; // Calculate error using last known good values
                 SerialCom->print(" | Err:"); SerialCom->print(error, 1);
                 SerialCom->print(" | Stable:"); SerialCom->print(stable_count);
                 SerialCom->println(); // Newline after full status print
            } else {
                SerialCom->println(" | Waiting for valid readings...");
            }
        }

        // Perform alignment logic only if BOTH sensors have provided at least one valid reading
        if (ir_left > 0 && ir_right > 0) {
            error = ir_left - ir_right; // Recalculate error just in case

            if (abs(error) <= tolerance) {
                // Within tolerance
                stable_count++;
                stop(); // Stop movement while checking stability
                 // Optional: Print stability increase immediately
                SerialCom->print("  Stable count increased: "); SerialCom->println(stable_count);
            } else {
                // Out of tolerance, need to rotate
                stable_count = 0; // Reset stability counter

                // Calculate turn speed based on error and Kp
                rotation_speed = Kp * abs(error); // Use absolute error for speed magnitude
                rotation_speed = constrain(rotation_speed, ORIENTATE_MIN_SPEED, ORIENTATE_MAX_SPEED);

                // Rotate based on the sign of the error (first code's logic)
                if (error > 0) { // Left is further away, turn counter-clockwise (left)
                    // Optional: Print rotation direction if not printing periodically
                    if(millis() - lastPrintTime > 10) SerialCom->println("  Rotating CCW...");
                    speed_val = rotation_speed; // Set speed for rotation
                    ccw();
                } else { // Right is further away (or equal, error <=0), turn clockwise (right)
                     // Optional: Print rotation direction if not printing periodically
                    if(millis() - lastPrintTime > 10) SerialCom->println("  Rotating CW...");
                    speed_val = rotation_speed; // Set speed for rotation
                    cw();
                }
                 // Reset print timer if movement occurs to potentially show next status sooner
                 lastPrintTime = millis();
            }
        } else {
            // One or both sensors haven't returned a valid reading yet, or returned an error
            // Keep stopped and wait for valid readings
            stop();
            stable_count = 0; // Cannot be stable without valid readings
            // Optional: Add a longer delay here if sensors persistently fail?
            // delay(100);
        }

        // Main loop delay - adjust as needed
        delay(LOOP_TIME_MS); // Or use delay(25); like second code if LOOP_TIME_MS isn't defined

    } while (stable_count < required_count);

    // Orientation complete or loop exited
    stop(); // Ensure robot is stopped

    // Final status message
    if (stable_count >= required_count) {
      SerialCom->println("Orientation Parallel Complete.");
    } else {
      // This case should ideally not be reached without a timeout or other break condition
      SerialCom->println("Orientation loop exited unexpectedly.");
    }
}

void shallowAngleCheck(char input) {
  float adjacent = 0;
  float opposite;
  float angle;
  float temp;
  int max_index = 10;
  myservo.write(90);
  delay(1000);
  for (int i = 0; i < max_index; i++) {
    temp = HC_SR04_range();
    if (temp != -1) {
      adjacent += temp;
    }
  }

  adjacent = adjacent / max_index;


  if (input == 'l') {
    myservo.write(180);
  } else {
    myservo.write(0);
  }
  delay(1000);
  opposite = HC_SR04_range();

  angle = atan(opposite / adjacent) * (180 / 3.1415926535897932385);
  SerialCom->print("Angle: ");
  SerialCom->println(angle);
  if (angle < 30) {
    if (input == 'l') {
      strafe_right();
      delay(350);
      ccw();
    } else {
      strafe_left();
      delay(350);
      cw();
    }

    delay(350);
    stop();
  }
  myservo.write(90);
}

void recalibrateGyro() {
  SerialCom->println("Recalibrating gyroscope...");
  delay(100);  // Allow time for the robot to stabilize

  float filter_cal = 0;
  float sum = 0;
  const int numReadings = 100;  // Number of readings to average

  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(gyroPin);  // Read raw gyro value
    filter_cal = kalmanFilter(analogRead(gyroPin), 20);
    delay(5);                    // Small delay between readings
  }

  // Calculate the new zero voltage
  gyroZeroVoltage = (sum / numReadings) * (gyroSupplyVoltage / 1023);  // Convert ADC to voltage

  SerialCom->print("New calibrated Zero Voltage: ");
  SerialCom->println(gyroZeroVoltage);

  currentAngle = 0;
}

double kalmanFilter(double U, double R) {

  // constants (static)
  // static const double R = 40; // noise covariance
  static const double H = 1.00; // measurement map scalar
  static double Q = 10; // initial estimated covariance
  static double P = 0; // initial error covariance (must be 0)
  static double U_hat = 0; // iniial estimated state (assume we don't know)
  static double K = 0; // initial kalman gain

  if(U == -1) {
    return -1; // return -1 if no value is found
  }

  // begin 
  K = P*H / (H * P * H + R); // calculate kalman gain, higher R means lower gain, but more filtered
  U_hat = U_hat + K * (U - H * U_hat); // update state estimate

  // update error covariance
  P = (1-K * H) * P + Q; // update error covariance

  return U_hat; // return estimated state
}      

// Function to update the current angle using the gyroscope
void updateCurrentAngle() {
  int rawValue = kalmanFilter(analogRead(gyroPin), 20);                                    // Read raw gyro value
  
  gyroRate = ((rawValue * gyroSupplyVoltage) / 1023) - gyroZeroVoltage;  // Calculate angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

  if (abs(angularVelocity) >= rotationThreshold) {
    float angleChange = angularVelocity * (T / 1000.0);  // Correct integration formula
    currentAngle += angleChange;
  }

  // Wrap the current angle within -180 - 180 degrees
  if (currentAngle < -180) currentAngle += 360;
  else if (currentAngle >= 180) currentAngle -= 360;

  // Debugging output
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}

// Function to check if the robot has reached the target angle
bool checkAngle(float targetAngle) {
  float angleDifference = abs(currentAngle - targetAngle);

  // Debugging output
  Serial.print("Target Angle: ");
  Serial.println(targetAngle);
  Serial.print("Angle Difference: ");
  Serial.println(angleDifference);

  // Check if the angle difference is within the threshold
  return angleDifference <= rotationThreshold;
}

// Updated rotate function
void rotate(int degrees) {
  // Gyro Calibration
  recalibrateGyro();

  int kp = 10;
  float targetAngle = currentAngle + degrees;  // Calculate the target angle
  // if (targetAngle < -180) targetAngle += 360;     // Wrap around if less than 0
  // if (targetAngle >= 180) targetAngle -= 360;  // Wrap around if greater than 359

  float motor_pwr = 100;

  float error = targetAngle - currentAngle;

  do {
    updateCurrentAngle();
    error = (targetAngle - currentAngle);
    motor_pwr = kp * error;
    speed_val = constrainPwr(abs(motor_pwr));
    // Determine rotation direction
    if (error > 0) {
      cw();  // Clockwise rotation
    } else {
      ccw();  // Counterclockwise rotation
    }

    // Rotate until the target angle is reached


    delay(T);  // Delay for the loop time
  } while (!checkAngle(targetAngle));

  stop();  // Stop the robot after reaching the target angle
  SerialCom->println("Rotation complete.");
}



// Convert raw sensor value to distance - don't use this for decisions
float calculateDistance(int rawValue) {
  return (rawValue - SENSOR_OFFSET) / SENSOR_SCALE;
}

float constrainPwr(float pwr) {
  float upper = 300;
  float lower = 75;

  if (pwr > upper) {
    return upper;
  } else if (pwr < lower) {
    return lower;
  }

  return pwr;
}

float constrainPwr(float pwr, float lower, float upper) {
  if (pwr > upper) {
    return upper;
  } else if (pwr < lower) {
    return lower;
  }

  return pwr;
}



void forwardUntil(int targetDistance, float kp) {
  // Set the speed for forward movement until robot is a set distance away
  speed_val = 100;
  float current_distance = 100;
  float buffer_dist = -1;
  float error;
  int count = 0;
  int required_count = 5;
  float tolerance = 1;


  // Tmeout feature to turn off after a certain time automatically
  unsigned long startTime = millis();

  do {
    if (millis() - startTime > TIMEOUT_DURATION) {
      Serial.println("Timeout reached. Stopping.");
      break;
    }

    error = current_distance - targetDistance;

    //  // Read the current distance from the front sensor
    //  int currentRawValue = sensor1.readRawValue();

    //  // Convert the raw value to distance
    //  float currentDistance = calculateDistance(currentRawValue);

    buffer_dist = HC_SR04_range();
    if (buffer_dist != -1) {
      current_distance = buffer_dist;
      Serial.print("Current Distance: ");
      Serial.println(current_distance);
    }


    // Check if the target distance has been reached
    //  if ((currentDistance <= targetDistance) &&
    //      (currentRawValue > SENSOR_MIN_THRESHOLD && currentRawValue < SENSOR_MAX_THRESHOLD)) {
    if (abs(error) <= tolerance) {
      Serial.print("Target Distance Reached: ");
      Serial.println(current_distance);
      count++;
    } else {
      count = 0;
    }
    speed_val = constrainPwr(kp * abs(error));
    // Move forward
    if (error > 0) {
      forward();
    } else {
      reverse();
    }
    delay(60);
  } while (count < required_count);

  // Stop the robot once the target distance is reached
  stop();
}



void move(float y_dist, float x_dist, float target_angle, int y_direction, int servo_direction) {
  float y_err, x_err, ang_err;
  float current_y, current_x, current_ang, buffer_dist_x;
  int y_pwr, x_pwr, ang_pwr;
  // float x_kp, y_kp, ang_kp;
  float ir_left, ir_right;
  int count = 0;
  int required_count = 2;

  // servo facing left = 1, right = -1

  // x_kp = 40;
  // x_kp = -40;
  // y_kp = 40;
  // ang_kp = 3;

  // y_tolerance = 2;
  // x_tolerance = 2;
  // ang_tolerance = 3;

  current_y = 200;
  // int wall_right = 1;
  // int servo_angle = myservo.read();
  // if (servo_angle < 90){
  //   wall_right = -1;
  // } 
  do {
    // // IR Sensor Check -- Y_dist Front and Back

    if (y_direction > 0) {
      ir_left = med_ir_dist(IR_LeftPIN);
      ir_right = med_ir_dist(IR_RightPIN);
      SerialCom->print("Left sensor: ");
      SerialCom->println(ir_left);
      SerialCom->print("Right sensor: ");
      SerialCom->println(ir_right);
    } else {
      ir_left = long_ir_dist(IR_LeftPIN_Back);
      ir_right = long_ir_dist(IR_RightPIN_Back);
      SerialCom->print("Left sensor: ");
      SerialCom->println(ir_left);
      SerialCom->print("Right sensor: ");
      SerialCom->println(ir_right);
    }

    if ((ir_left != -1)) {
      if ((ir_right != -1)) {                  // either one is reading
        current_y = (ir_left + ir_right) / 2;  //if one of them isn't reading need to change
      } else {
        current_y = (ir_left);
      }
    } else if ((ir_right != -1)) {
      // either one is reading
      current_y = (ir_right);
    } else {
      current_y = 200;
    }


    // Ultra sonic
    buffer_dist_x = HC_SR04_range();
    if (buffer_dist_x != -1) {
      current_x = buffer_dist_x;
      // Serial.print("Current Distance: ");
      // Serial.println(current_distance);
    }

    // Gyro
    updateCurrentAngle();


    // Error Values
    // y_err = (y_dist / abs(y_dist)) * (abs(y_dist) - current_y); 
    y_err = -1*(y_dist - current_y);
    x_err = x_dist - current_x;
    ang_err = target_angle - currentAngle;
    SerialCom->print("Y err: ");
    SerialCom->println(y_err);
    SerialCom->print("X err: ");
    SerialCom->println(x_err);
    SerialCom->print("Ang err: ");
    SerialCom->println(ang_err);
    SerialCom->print("Y current: ");
    SerialCom->println(current_y);
    SerialCom->print("X current: ");
    SerialCom->println(current_x);
    SerialCom->print("Ang current: ");
    SerialCom->println(currentAngle);

    // Power!!!!
    // y_pwr = constrainPwr((y_kp * y_err), -200, 200);
    y_pwr = (y_err/abs(y_err))*constrainPwr(abs(y_kp * y_err), 30, 250);
    // x_pwr = constrainPwr((x_kp * x_err), -200, 200);
    x_pwr = (x_err/abs(x_err))*constrainPwr(abs(x_kp * x_err), 30, 200);
    ang_pwr = constrainPwr((ang_kp * ang_err), -100, 100);
    // ang_pwr = (ang_err/abs(ang_err))*constrainPwr(abs(ang_kp * ang_err), 75, 100);


    //forward convention, y = forward, x = strafe right, r = clockwise
    left_font_motor.writeMicroseconds(1500 + (y_direction)*y_pwr + (servo_direction)*x_pwr + ang_pwr);
    left_rear_motor.writeMicroseconds(1500 + (y_direction)*y_pwr - (servo_direction)*x_pwr + ang_pwr);
    right_rear_motor.writeMicroseconds(1500 - (y_direction)*y_pwr - (servo_direction)*x_pwr + ang_pwr);
    right_font_motor.writeMicroseconds(1500 - (y_direction)*y_pwr + (servo_direction)*x_pwr + ang_pwr);

    if ((abs(y_err) >= y_tolerance) || (abs(x_err) >= x_tolerance)) {
      count++;
    } else {
      count = 0;
    }
      // while ((abs(y_err) >= y_tolerance) || (abs(x_err) >= x_tolerance));
    // while (count < required_count);
    //|| (abs(ang_err) >= ang_tolerance)
    delay(80);
  } while (count < required_count);
  stop();
}


void setMoveTolerance(float x, float y, float ang) {
  y_tolerance = x; 
  x_tolerance = y; 
  ang_tolerance = ang;

}

#ifndef NO_HC - SR04
float HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return -1;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return -1;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
    return cm;
  }
}
#endif

void Analog_Range_A4() {
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading() {
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w':  //Move Forward
      case 'W':
        forward();
        SerialCom->println("Forward");
        break;
      case 's':  //Move Backwards
      case 'S':
        reverse();
        SerialCom->println("Backwards");
        break;
      case 'q':  //Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e':  //Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a':  //Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd':  //Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-':  //Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }
  }
}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
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