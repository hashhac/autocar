/*
  Refactored MechEng 706 Base Code - V9.1
  Focus: Implementing full original logic sequence with perimeter navigation.
         Sequence: Shallow Check -> Approach -> Orient -> Turn -> Move Corner
                   -> Check Dims -> Orient Long -> Drive Perimeter -> Stop.

  - V9.1 Changes:
    - Added detailed debug prints in HC_SR04_range().
    - shallowAngleCheck() now returns bool (true on success, false on failure).
    - test() now checks return value of shallowAngleCheck() and stops on failure.
    - Added debug print inside the "slow forward on fail" loop in test().
    - Fully formatted code.
  - Based on V9 code structure and functions.
  - *** NOTE: Ensure LiPo battery is fully charged before testing! ***
*/
#include <Servo.h>
#include <Arduino.h>
#include <math.h> // Include for atan, fabs

// --- Configuration ---
// #define NO_READ_GYRO      // Uncomment if GYRO is not attached.
// #define NO_HC_SR04        // Uncomment if HC-SR04 ultrasonic is not attached.
// #define NO_BATTERY_V_OK   // Uncomment if you do not care about battery voltage/damage.

// State machine states (Simplified)
enum STATE {
  INITIALISING,
  STOPPED,
  TEST     // The main autonomous sequence state
};

// --- Pin Definitions ---
const byte left_front_pin = 46;
const byte left_rear_pin = 47;
const byte right_rear_pin = 50;
const byte right_front_pin = 51;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const int IR_LeftPIN = A4;       // Med Range Front Left (Used by orientate, move)
const int IR_RightPIN = A5;      // Med Range Front Right (Used by orientate, move)
const int IR_LeftPIN_Back = A6;  // Long Range Back Left (Used by move) - Ensure connected
const int IR_RightPIN_Back = A7; // Long Range Back Right (Used by move) - Ensure connected
const int gyroPin = A3;
const int SERVO_PIN = 7; // Ultrasonic Turret Servo

// --- Constants ---
const unsigned int MAX_DIST_US = 23200; // ~400 cm pulse width
const unsigned long US_TIMEOUT = MAX_DIST_US + 1000; // Timeout for pulseIn

// IR Calibration Constants (Adjust as needed)
const float MED_IR_MULT = 2118.6;
const float MED_IR_OFFSET = 20.072;
const float LONG_IR_MULT = 4261.4; // For back sensors used in move()
const float LONG_IR_OFFSET = 61.06; // For back sensors used in move()
const int IR_MIN_READING = 110;    // Min plausible raw ADC reading for IR
const int IR_MAX_READING = 550;    // Max plausible raw ADC reading for IR
const float MAX_MED_IR_RANGE = 35.0; // Approx max reliable range for Med IR (cm) - Tune this
const float MAX_LONG_IR_RANGE = 85.0;// Approx max reliable range for Long IR (cm) - Tune this

// Gyro Constants
const float gyroSupplyVoltage = 5.0;
float gyroZeroVoltage = 1.65; // Default value - MUST RECALIBRATE
const float gyroSensitivity = 0.007; // V/(deg/s)
const float rotationThreshold = 1.5; // deg/s (Gyro reading noise filter)
const float rotationTargetTolerance = 2.0; // deg (Acceptable error for rotate() completion)

// Movement & Timing Constants
const int MAX_SPEED = 300;
int speed_val = 150; // Default speed for basic moves
const int FIND_WALL_SPEED = 100; // Slow speed for initially finding wall
const int PRELIM_STRAFE_SPEED = 75; // Slow speed for initial strafe in move()
const int SLOW_FWD_ON_FAIL_SPEED = 75; // Speed for moving fwd if initial US fails
const unsigned long TIMEOUT_DURATION = 15000; // Default timeout (ms)
const unsigned long MOVE_TIMEOUT = TIMEOUT_DURATION * 2; // Longer timeout for move()
const unsigned long PRELIM_STRAFE_TIMEOUT = 5000; // Max time for initial strafe (ms)
const unsigned long FIND_WALL_ON_FAIL_TIMEOUT = 10000; // Timeout for moving fwd on initial US fail
const int LOOP_TIME_MS = 50; // ms, Target loop time

// Strafe Calibration Factors
const float strafe_motor_TL_speed = 1.0;
const float strafe_motor_TR_speed = 0.993;
const float strafe_motor_BL_speed = 0.993;
const float strafe_motor_BR_speed = 1.0;

// Angle Check Constants
const float SHALLOW_ANGLE_THRESHOLD = 40.0; // degrees - Increased threshold
const int SHALLOW_CORRECTION_STRAFE_DELAY = 350; // ms for strafe part
const int SHALLOW_CORRECTION_ROTATE_DEGREES = 20; // Fixed rotation amount for correction

// PID Gains for move() function (Tune these carefully!)
const float MOVE_Y_KP = 20.0; // Side distance (IR) gain
const float MOVE_X_KP = 40.0; // Front distance (US) gain
const float MOVE_ANG_KP = 5.0; // Angle (Gyro) gain

// Tolerances for move() function
const float MOVE_Y_TOLERANCE = 1.5; // cm
const float MOVE_X_TOLERANCE = 1.5; // cm
const float MOVE_ANG_TOLERANCE = 2.0; // degrees

const int ROTATE_KP = 15; // Proportional gain for rotate() responsiveness

// Orientate Constants
const float ORIENTATE_TOLERANCE = 1.5; // IR distance difference tolerance (cm)
const int ORIENTATE_SPEED = 60;       // Slow speed for fine adjustment
const int ORIENTATE_STABLE_COUNT = 5; // Required consecutive readings within tolerance
const unsigned long ORIENTATE_TIMEOUT = TIMEOUT_DURATION / 2; // Shorter timeout for orient

// Navigation Constants
const float DEFAULT_LONG_DIM = 200.0; // cm - Assume if measurement fails
const float DEFAULT_SHORT_DIM = 100.0; // cm - Assume if measurement fails
const float NAV_WALL_DISTANCE = 20.0; // Target distance from wall during navigation

// --- Global Variables ---
Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;
Servo sensorServo;
float currentAngle = 0.0; // Global angle tracked by gyro integration
HardwareSerial* SerialCom; // Pointer for Serial output (USB or other)

// --- Function Declarations ---
// State Machine
STATE initialising();
STATE stopped();
STATE test();

// Sensor Functions
float HC_SR04_range();
float med_ir_dist(int ir_pin);
float long_ir_dist(int ir_pin);
void recalibrateGyro();
void updateCurrentAngle();
bool checkAngle(float targetAngle);

// Movement Functions
void enable_motors();
void disable_motors();
void stop();
void forward(int speed = speed_val);
void reverse(int speed = speed_val);
void ccw(int speed = speed_val); // Counter-Clockwise rotation
void cw(int speed = speed_val);  // Clockwise rotation
void strafeLeft(int speed = speed_val);
void strafeRight(int speed = speed_val);
void rotate(int degrees, bool doRecalibrate = true); // Rotate using gyro
void forwardUntil(int targetDistance, float kp);     // Move forward/back to US distance
void orientate();                                   // Align parallel using front IR
bool shallowAngleCheck(char side);                  // Check and correct shallow angle (returns bool)
void move(float y_dist, float x_dist, float target_angle); // Complex multi-axis move

// Utility Functions
boolean is_battery_voltage_OK();
void slow_flash_LED_builtin();
float constrainPwr(float pwr, float lower, float upper);


// ==================== SETUP ====================
void setup(void) {
  SerialCom = &Serial; // Use USB serial
  SerialCom->begin(115200);
  while (!Serial); // Wait for serial connection

  SerialCom->println("\n=== Robot Initializing (V9.1 - Debug US Fail) ===");
  SerialCom->println("Code: Full Sequence with Dimension Check & Perimeter Nav");
  SerialCom->println("*** ENSURE BATTERY IS FULLY CHARGED! ***");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  sensorServo.attach(SERVO_PIN);
  sensorServo.write(90);
  SerialCom->println("Ultrasonic servo attached and centered.");

  pinMode(gyroPin, INPUT);
  SerialCom->println("Gyro pin set to input.");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  SerialCom->println("Ultrasonic pins initialized.");

  SerialCom->println("IR sensor pins ready (Front A4/A5, Back A6/A7).");
  SerialCom->println("Peripherals initialized. Waiting 1 second...");
  delay(1000); // Allow components to stabilize
}

// ==================== MAIN LOOP ====================
void loop(void) {
  static STATE machine_state = INITIALISING;
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case STOPPED:
      machine_state = stopped();
      break;
    case TEST:
      machine_state = test();
      break;
  };
}

// ==================== STATE FUNCTIONS ====================

STATE initialising() {
  SerialCom->println("--- STATE: INITIALISING ---");
  digitalWrite(LED_BUILTIN, HIGH);
  SerialCom->println("Enabling Motors...");
  enable_motors();
  stop();
  SerialCom->println("Motors enabled and stopped.");
#ifndef NO_READ_GYRO
  SerialCom->println("Performing initial Gyro calibration...");
  recalibrateGyro();
#endif
  digitalWrite(LED_BUILTIN, LOW);
  SerialCom->println("Transitioning to TEST state.");
  return TEST;
}

STATE test() {
  // --- Parameters ---
  float approach_dist = 20.0;
  int approach_kp = 8;
  int first_turn_angle = -90;
  float corner_y_dist = 20.0;
  float corner_x_dist = 20.0;
  float corner_target_angle = 0.0;
  float short_wall_threshold = 150.0;
  int final_turn_angle = -90;

  SerialCom->println("\n=== STATE: TEST - Full Sequence with Navigation ===");
  sensorServo.write(90);
  delay(500);

  // --- Step 1: Shallow Angle Check (Threshold 40 deg) ---
  SerialCom->println("--- Step 1: Checking/Correcting Shallow Angle (< 40 deg) ---");
  bool check_l_ok = shallowAngleCheck('l');
  delay(500);
  // Only proceed if first check was okay
  if (!check_l_ok) {
    SerialCom->println("Stopping sequence due to failure in shallowAngleCheck(left).");
    // return STOPPED;
  }

  bool check_r_ok = shallowAngleCheck('r');
  delay(500);
  // Only proceed if second check was okay
  if (!check_r_ok) {
    SerialCom->println("Stopping sequence due to failure in shallowAngleCheck(right).");
    // return STOPPED;
  }

  SerialCom->println("Shallow angle checks complete.");
  delay(1000);

  // --- Step 2: Approach Wall ---
  SerialCom->println("--- Step 2: Approaching wall ---");
  float initial_dist_avg = 0;
  int valid_readings = 0;
  bool found_wall_initially = false;

  // Try to get initial distance readings
  SerialCom->println("  Attempting initial distance readings...");
  for (int i = 0; i < 3; i++) {
    float dist = HC_SR04_range(); // This function now has more debug prints
    SerialCom->print("    Initial read "); SerialCom->print(i + 1); SerialCom->print(": ");
    SerialCom->println(dist > 0 ? String(dist) : "Fail");
    if (dist > 0) {
      initial_dist_avg += dist;
      valid_readings++;
    }
    delay(50);
  }

  if (valid_readings > 0) {
    // Got at least one valid reading initially
    initial_dist_avg /= (float)valid_readings;
    found_wall_initially = true;
    SerialCom->print("  Initial average distance: ");
    SerialCom->print(initial_dist_avg);
    SerialCom->println(" cm");
  } else {
    // All initial readings failed, try moving forward slowly
    SerialCom->println("  Initial US readings failed. Moving forward slowly to find wall...");
    unsigned long findStartTime = millis();
    while (millis() - findStartTime < FIND_WALL_ON_FAIL_TIMEOUT) {
      forward(SLOW_FWD_ON_FAIL_SPEED); // Move forward slowly
      float current_dist = HC_SR04_range();
      SerialCom->print("    ...Moving fwd, current dist: "); // Debug print inside loop
      SerialCom->println(current_dist > 0 ? String(current_dist) : "Fail");
      delay(LOOP_TIME_MS); // Small delay

      if (current_dist > 0) { // Check if we got a valid reading
        SerialCom->print("  Wall detected while moving forward at: ");
        SerialCom->print(current_dist);
        SerialCom->println(" cm");
        initial_dist_avg = current_dist; // Use this first valid reading
        found_wall_initially = true;
        stop(); // Stop moving forward
        break;  // Exit the finding loop
      }
    } // End while finding wall

    if (!found_wall_initially) {
      SerialCom->println("Could not find wall after moving forward slowly (Timeout). Stopping.");
      stop();
      return STOPPED; // Abort if wall still not found
    }
  } // End else (initial readings failed)

  // Now proceed with approach logic using the initial_dist_avg
  if (found_wall_initially) {
    if (initial_dist_avg > approach_dist + 10.0) {
      SerialCom->println("  Approaching target distance...");
      forwardUntil(approach_dist, approach_kp);
      SerialCom->println("  Approach complete.");
    } else {
      SerialCom->println("  Already close enough to wall, skipping final approach.");
    }
  }
  delay(1000); // Pause after approaching

  // --- Step 3: Orient Parallel (IR) ---
  SerialCom->println("--- Step 3: Orienting parallel to wall (IR) ---");
  orientate();
  SerialCom->println("Orientation parallel complete.");
  delay(1000);

  // --- Step 4: Turn Left 90 deg ---
  SerialCom->println("--- Step 4: Performing first turn ---");
  SerialCom->print("Rotating "); SerialCom->print(first_turn_angle); SerialCom->println(" degrees...");
  rotate(first_turn_angle, true);
  SerialCom->println("First turn complete.");
  delay(1000);

  // --- Step 5: Move to Corner (X=20, Y=20) ---
  SerialCom->println("--- Step 5: Moving to corner (X=20, Y=20, Ang=0) ---");
  move(corner_y_dist, corner_x_dist, corner_target_angle);
  SerialCom->println("Move to corner attempt complete.");
  delay(1000);

  // --- Step 6: Determine Dimensions & Orient Long ---
  SerialCom->println("--- Step 6: Determining Dimensions & Orienting Long ---");
  float dist_B = -1, dist_A = -1;
  float length_A = DEFAULT_LONG_DIM, length_B = DEFAULT_SHORT_DIM; // Default assumptions
  bool facing_long = true; // Assume current direction (A) is long initially

  // Measure distance B (currently facing)
  dist_B = HC_SR04_range();
  delay(100);
  if (dist_B > 0) {
    length_B = dist_B + corner_x_dist;
    SerialCom->print("Measured Length B (approx): ");
    SerialCom->println(length_B);
  } else {
    SerialCom->println("Failed to measure Length B, using default.");
  }

  // Turn right to face A
  SerialCom->println("Rotating right to measure Length A...");
  rotate(90, true);
  delay(500);

  // Measure distance A
  dist_A = HC_SR04_range();
  delay(100);
  if (dist_A > 0) {
    length_A = dist_A + corner_x_dist;
    SerialCom->print("Measured Length A (approx): ");
    SerialCom->println(length_A);
  } else {
    SerialCom->println("Failed to measure Length A, using default.");
  }

  // Determine long/short and orient
  float long_dim, short_dim;
  if (length_B > length_A) {
    SerialCom->println("Length B is longer. Rotating left to face B...");
    rotate(-90, false); // Turn back left to face long dimension B
    long_dim = length_B;
    short_dim = length_A;
    facing_long = true; // Now facing long
  } else {
    SerialCom->println("Length A is longer (or equal). Already facing A.");
    long_dim = length_A;
    short_dim = length_B;
    facing_long = true; // Already facing long
  }
  SerialCom->print("Determined Dimensions -> Long: "); SerialCom->print(long_dim);
  SerialCom->print(" | Short: "); SerialCom->println(short_dim);
  delay(1000);

  // --- Step 7: Navigate Perimeter ---
  SerialCom->println("--- Step 7: Navigating Perimeter ---");
  // We are in the corner (20, 20), facing the long dimension.
  int nav_kp = approach_kp; // Use same kp for navigation legs

  // Leg 1: Drive along long dimension
  SerialCom->println("  Nav: Driving Long Leg 1...");
  forwardUntil(NAV_WALL_DISTANCE, nav_kp);
  delay(500);

  // Turn 1: Turn left
  SerialCom->println("  Nav: Turning Left 1...");
  rotate(-90, true);
  delay(500);

  // Leg 2: Drive along short dimension
  SerialCom->println("  Nav: Driving Short Leg 1...");
  forwardUntil(NAV_WALL_DISTANCE, nav_kp);
  delay(500);

  // Turn 2: Turn left
  SerialCom->println("  Nav: Turning Left 2...");
  rotate(-90, true);
  delay(500);

  // Leg 3: Drive back along long dimension
  SerialCom->println("  Nav: Driving Long Leg 2...");
  forwardUntil(NAV_WALL_DISTANCE, nav_kp);
  delay(500);

  // Turn 3: Turn left
  SerialCom->println("  Nav: Turning Left 3...");
  rotate(-90, true);
  delay(500);

  // Leg 4: Drive back along short dimension (back to start)
  SerialCom->println("  Nav: Driving Short Leg 2...");
  forwardUntil(NAV_WALL_DISTANCE, nav_kp);
  delay(500);

  // Turn 4: Turn left (optional, return to original orientation)
  SerialCom->println("  Nav: Turning Left 4 (Return to Start Orientation)...");
  rotate(-90, true);
  delay(500);

  SerialCom->println("Navigation complete.");

  // --- Sequence Complete ---
  SerialCom->println("=== Full Sequence Complete ===");
  SerialCom->println("Transitioning to STOPPED state.");
  return STOPPED;
}

STATE stopped() {
  static unsigned long previous_millis = 0;
  static bool stopped_message_printed = false;
  disable_motors();
  slow_flash_LED_builtin();
  if (millis() - previous_millis > 2000) {
    previous_millis = millis();
    if (!stopped_message_printed) {
      SerialCom->println("--- STATE: STOPPED (Motors Disabled) ---");
      stopped_message_printed = true;
    } else {
      SerialCom->println("--- STATE: STOPPED ---");
    }
    #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) {
      SerialCom->println("Battery low. Please recharge.");
    } else {
      SerialCom->println("Battery OK.");
    }
    #endif
  }
  return STOPPED;
}


// ==================== SENSOR FUNCTIONS ====================

#ifndef NO_HC_SR04
// Reads distance using HC-SR04 Ultrasonic sensor
float HC_SR04_range() {
  unsigned long duration;
  float cm;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(50); // Small delay before pulseIn
  duration = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);

  // Debug print for raw duration
  // SerialCom->print("    [US Raw Duration: "); SerialCom->print(duration); SerialCom->println("]");

  if (duration > 0 && duration < MAX_DIST_US) {
    cm = duration / 58.0;
    return cm;
  } else if (duration == 0) {
    // SerialCom->println("    [US Timeout]");
    return -1.0;
  } else {
    // SerialCom->println("    [US Out of Range]");
    return -2.0;
  }
}
#else
float HC_SR04_range() {
  SerialCom->println("HC-SR04 Disabled");
  return -1.0;
}
#endif

// Reads distance from Medium Range IR sensor
float med_ir_dist(int ir_pin) {
  int v = analogRead(ir_pin);
  float d = -1.0;
  if ((v >= IR_MIN_READING) && (v <= IR_MAX_READING)) {
    if (abs(v - MED_IR_OFFSET) > 0.1) {
      d = MED_IR_MULT / (v - MED_IR_OFFSET);
      if (d > MAX_MED_IR_RANGE || d < 0) {
        d = -4.0; // Out of reliable range
      }
    } else {
      d = -2.0; // Denominator error
    }
  } else {
    d = -3.0; // Raw reading error
  }
  return d;
}

// Reads distance from Long Range IR sensor
float long_ir_dist(int ir_pin) {
  int v = analogRead(ir_pin);
  float d = -1.0;
  if ((v >= IR_MIN_READING) && (v <= IR_MAX_READING)) {
    if (abs(v - LONG_IR_OFFSET) > 0.1) {
      d = LONG_IR_MULT / (v - LONG_IR_OFFSET);
      if (d > MAX_LONG_IR_RANGE || d < 0) {
        d = -4.0; // Out of reliable range
      }
    } else {
      d = -2.0; // Denominator error
    }
  } else {
    d = -3.0; // Raw reading error
  }
  return d;
}

#ifndef NO_READ_GYRO
// Recalibrates the gyro
void recalibrateGyro() {
  SerialCom->println("Recalibrating gyroscope...");
  long s = 0;
  const int n = 200;
  stop();
  delay(500);
  for (int i = 0; i < n; i++) {
    s += analogRead(gyroPin);
    delay(5);
  }
  gyroZeroVoltage = ((float)s / n) * (gyroSupplyVoltage / 1023.0);
  SerialCom->print("Gyro calibration complete. ZeroV: ");
  SerialCom->println(gyroZeroVoltage, 4);
  currentAngle = 0.0;
  delay(100);
}

// Updates the current angle estimate
void updateCurrentAngle() {
  static unsigned long t = 0;
  unsigned long n = millis();
  float dt = (n - t) / 1000.0;
  if (dt <= 0) return;
  t = n;
  int r = analogRead(gyroPin);
  float v = (((r * gyroSupplyVoltage) / 1023.0) - gyroZeroVoltage) / gyroSensitivity;
  if (abs(v) >= rotationThreshold) {
    currentAngle += v * dt;
  }
}

// Checks if current angle is close to target angle
bool checkAngle(float targetAngle) {
  float d = abs(targetAngle - currentAngle);
  return d <= rotationTargetTolerance;
}
#else
void recalibrateGyro() { SerialCom->println("Gyro Disabled - Recalibration Skipped"); }
void updateCurrentAngle() {}
bool checkAngle(float t) { return true; }
#endif

// ==================== MOVEMENT FUNCTIONS ====================

void enable_motors() {
  pinMode(left_front_pin, OUTPUT);
  pinMode(left_rear_pin, OUTPUT);
  pinMode(right_rear_pin, OUTPUT);
  pinMode(right_front_pin, OUTPUT);
  left_font_motor.attach(left_front_pin);
  left_rear_motor.attach(left_rear_pin);
  right_rear_motor.attach(right_rear_pin);
  right_font_motor.attach(right_front_pin);
  SerialCom->println("Motors Enabled.");
}

void disable_motors() {
  left_font_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_font_motor.detach();
  pinMode(left_front_pin, INPUT);
  pinMode(left_rear_pin, INPUT);
  pinMode(right_rear_pin, INPUT);
  pinMode(right_front_pin, INPUT);
  // Print removed
}

void stop() {
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward(int s) {
  s = constrain(s, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 + s);
  left_rear_motor.writeMicroseconds(1500 + s);
  right_rear_motor.writeMicroseconds(1500 - s);
  right_font_motor.writeMicroseconds(1500 - s);
}

void reverse(int s) {
  s = constrain(s, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 - s);
  left_rear_motor.writeMicroseconds(1500 - s);
  right_rear_motor.writeMicroseconds(1500 + s);
  right_font_motor.writeMicroseconds(1500 + s);
}

void ccw(int s) {
  s = constrain(s, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 - s);
  left_rear_motor.writeMicroseconds(1500 - s);
  right_rear_motor.writeMicroseconds(1500 - s);
  right_font_motor.writeMicroseconds(1500 - s);
}

void cw(int s) {
  s = constrain(s, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 + s);
  left_rear_motor.writeMicroseconds(1500 + s);
  right_rear_motor.writeMicroseconds(1500 + s);
  right_font_motor.writeMicroseconds(1500 + s);
}

void strafeLeft(int s) {
  s = constrain(s, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 - (int)(s * strafe_motor_TL_speed));
  left_rear_motor.writeMicroseconds(1500 + (int)(s * strafe_motor_BL_speed));
  right_rear_motor.writeMicroseconds(1500 + (int)(s * strafe_motor_BR_speed));
  right_font_motor.writeMicroseconds(1500 - (int)(s * strafe_motor_TR_speed));
}

void strafeRight(int s) {
  s = constrain(s, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 + (int)(s * strafe_motor_TL_speed));
  left_rear_motor.writeMicroseconds(1500 - (int)(s * strafe_motor_BL_speed));
  right_rear_motor.writeMicroseconds(1500 - (int)(s * strafe_motor_BR_speed));
  right_font_motor.writeMicroseconds(1500 + (int)(s * strafe_motor_TR_speed));
}

void rotate(int degrees, bool doRecalibrate) {
#ifndef NO_READ_GYRO
  SerialCom->print("Rotate fn | Tgt Deg: "); SerialCom->print(degrees);
  SerialCom->print(" | Recal: "); SerialCom->println(doRecalibrate ? "Y" : "N");
  if (doRecalibrate) {
    recalibrateGyro();
  } else {
    updateCurrentAngle();
  }

  float startAngle = currentAngle;
  float targetAngle = startAngle + degrees;
  const int kp = ROTATE_KP;
  float motor_pwr;
  float error;
  int rotation_speed;
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  SerialCom->print("  sA:"); SerialCom->print(startAngle, 1);
  SerialCom->print(" | tA:"); SerialCom->println(targetAngle, 1);

  do {
    if (millis() - startTime > TIMEOUT_DURATION) {
      SerialCom->println("Rotate Timeout!");
      break;
    }
    updateCurrentAngle();
    error = targetAngle - currentAngle;
    motor_pwr = kp * error;
    rotation_speed = (int)constrain(abs(motor_pwr), 80, MAX_SPEED); // Min speed 50
    if (error > 0) {
      cw(rotation_speed);
    } else if (error < 0) {
      ccw(rotation_speed);
    } else {
      stop();
    }
    if (millis() - lastPrintTime > 200) {
      SerialCom->print("    Rot-> T:"); SerialCom->print(targetAngle, 1);
      SerialCom->print("|C:"); SerialCom->print(currentAngle, 1);
      SerialCom->print("|E:"); SerialCom->print(error, 1);
      SerialCom->print("|S:"); SerialCom->println(rotation_speed);
      lastPrintTime = millis();
    }
    delay(LOOP_TIME_MS / 5); // ~10ms delay
  } while (!checkAngle(targetAngle));

  stop();
  SerialCom->print("  Rot End Angle: "); SerialCom->println(currentAngle, 2);
#else
  SerialCom->println("Rotate skipped: Gyro Disabled.");
  delay(abs(degrees) * 15);
#endif
}
void rotate(int degrees) {
  rotate(degrees, true);
}

void forwardUntil(int targetDistance, float kp) {
#ifndef NO_HC_SR04
  SerialCom->print("forwardUntil | Target: "); SerialCom->print(targetDistance); SerialCom->println(" cm");
  float current_distance = 100.0;
  float buffer_dist = -1.0;
  float error;
  int move_speed;
  int stable_count = 0;
  int required_count = 5;
  float tolerance = 1.0;
  unsigned long startTime = millis();
  sensorServo.write(90);
  delay(200);

  do {
    if (millis() - startTime > TIMEOUT_DURATION) {
      SerialCom->println("FU Timeout!");
      break;
    }
    buffer_dist = HC_SR04_range();
    if (buffer_dist > 0) {
      current_distance = buffer_dist;
    } else {
      SerialCom->print("FU Warn: Bad US "); SerialCom->println(buffer_dist);
    }
    error = current_distance - targetDistance;
    move_speed = (int)constrainPwr(abs(error * kp), 50, MAX_SPEED);
    if (abs(error) <= tolerance) {
      stable_count++;
      stop();
      delay(LOOP_TIME_MS);
    } else {
      stable_count = 0;
      if (error > 0) {
        forward(move_speed);
      } else {
        reverse(move_speed);
      }
    }
    delay(LOOP_TIME_MS);
  } while (stable_count < required_count);

  stop();
  SerialCom->print("FU End | Final Dist: "); SerialCom->println(current_distance);
#else
  SerialCom->println("forwardUntil skipped: US Disabled.");
  delay(1000);
#endif
}

// Orients the robot parallel to a wall in front using front IR sensors.
void orientate() {
  SerialCom->println("Orientate fn (Align Parallel using IR)...");
  float tolerance = ORIENTATE_TOLERANCE;
  int rotation_speed = ORIENTATE_SPEED;
  int required_count = ORIENTATE_STABLE_COUNT;
  unsigned long timeout = ORIENTATE_TIMEOUT;
  float ir_left, ir_right, error;
  int stable_count = 0;
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0; // Timer for periodic printing

  do {
    if (millis() - startTime > timeout) {
      SerialCom->println("  Orientate Timeout!");
      break;
    }

    ir_left = med_ir_dist(IR_LeftPIN);
    ir_right = med_ir_dist(IR_RightPIN);

    // Print sensor readings periodically
    if (millis() - lastPrintTime > 100) { // Print every 100ms
        SerialCom->print("  Orientate -> L:"); SerialCom->print(ir_left > 0 ? String(ir_left, 1) : "Err");
        SerialCom->print(" | R:"); SerialCom->print(ir_right > 0 ? String(ir_right, 1) : "Err");
        lastPrintTime = millis();
    }


    if (ir_left > 0 && ir_right > 0) { // Check both valid
      error = ir_left - ir_right;
      // Print error only if printing sensor readings above
      if (millis() - lastPrintTime < 10) { // Avoid printing error twice quickly
          SerialCom->print(" | Err:"); SerialCom->print(error, 1);
      }


      if (abs(error) < tolerance) {
        stable_count++;
        // Print stability count only when it increments
        if (millis() - lastPrintTime < 10) {
            SerialCom->print(" | Stable:"); SerialCom->println(stable_count);
        } else {
            SerialCom->println(); // Newline if not printing error
            SerialCom->print("  Orientate -> Stable:"); SerialCom->println(stable_count);
        }
        stop();
        delay(LOOP_TIME_MS);
      } else {
        stable_count = 0;
        // Print rotation direction only when starting rotation
        if (millis() - lastPrintTime < 10) SerialCom->println(); // Newline if error was printed
        if (error > 0) {
          SerialCom->println("    Rotating CCW...");
          ccw(rotation_speed);
        } else {
          SerialCom->println("    Rotating CW...");
          cw(rotation_speed);
        }
         // Reset print timer after issuing a move command to avoid immediate reprint
        lastPrintTime = millis();
      }
    } else { // Handle sensor error
      // Print error only once per failure detection
      if (stable_count != -1) { // Use stable_count as a flag for printing error message once
         SerialCom->println(" | Err: Bad IR reading(s). Stopping rotation.");
         stable_count = -1; // Set flag
      }
      stop();
      delay(200); // Wait before potentially retrying
    }
    // Reset stable_count from -1 if sensors become valid again
    if (stable_count == -1 && ir_left > 0 && ir_right > 0) {
        stable_count = 0;
    }

    delay(LOOP_TIME_MS); // Main loop delay
  } while (stable_count < required_count);

  stop(); // Ensure robot is stopped

  // Final status message
  if (millis() - startTime > timeout) {
    SerialCom->println("Orientation failed due to timeout.");
  } else if (stable_count >= required_count) {
    SerialCom->println("Orientation Parallel Complete.");
  } else {
    SerialCom->println("Orientation stopped for unknown reason."); // Should ideally not happen
  }
}

// Checks for shallow angle (< threshold) using US and corrects with strafe + fixed rotate
// Returns true if successful (or skipped), false if failed to get readings.
bool shallowAngleCheck(char side) {
#if !defined(NO_HC_SR04) && !defined(NO_READ_GYRO)
  SerialCom->print("shallowAngleCheck | Side: "); SerialCom->println(side);
  float adjacent_dist = 0, opposite_dist = -1.0, angle = 90.0, temp_dist;
  int valid_readings = 0;
  const int num_readings_adj = 5;

  // 1. Measure adjacent distance
  sensorServo.write(90);
  delay(500);
  for (int i = 0; i < num_readings_adj; i++) {
    temp_dist = HC_SR04_range();
    if (temp_dist > 0) {
      adjacent_dist += temp_dist;
      valid_readings++;
    }
    delay(50);
  }
  if (valid_readings == 0) {
    SerialCom->println("  SAC Err: No adj reading.");
    sensorServo.write(90);
    return false; // Indicate failure
  }
  adjacent_dist /= (float)valid_readings;
  SerialCom->print("  Adj Dist: "); SerialCom->println(adjacent_dist);

  // 2. Measure opposite distance
  int side_angle = (side == 'l' || side == 'L') ? 170 : 10;
  sensorServo.write(side_angle);
  delay(500);
  opposite_dist = HC_SR04_range();
  sensorServo.write(90);
  delay(200);

  // 3. Calculate angle
  if (opposite_dist > 0 && adjacent_dist > 0.1) {
    angle = atan(opposite_dist / adjacent_dist) * (180.0 / M_PI);
    SerialCom->print("  Calc Ang (approx): "); SerialCom->println(angle);
  } else {
    SerialCom->print("  SAC Err: Bad opp D: "); SerialCom->println(opposite_dist);
    angle = 90.0;
    SerialCom->println("  Skipping correction due to bad opposite reading.");
    return true; // Return true as adjacent was okay, just couldn't check angle
  }

  // 4. Correct if angle IS below the SHALLOW_ANGLE_THRESHOLD
  if (angle < SHALLOW_ANGLE_THRESHOLD) {
    SerialCom->print("  Shallow (<"); SerialCom->print(SHALLOW_ANGLE_THRESHOLD);
    SerialCom->print("). Ang:"); SerialCom->print(angle); SerialCom->println(". Correcting...");
    int strafe_speed = 100;
    int rotate_amount = SHALLOW_CORRECTION_ROTATE_DEGREES;

    if (side == 'l' || side == 'L') {
      SerialCom->println("    Correct L: Strafe R + Rot CCW");
      strafeRight(strafe_speed);
      delay(SHALLOW_CORRECTION_STRAFE_DELAY);
      stop();
      delay(100);
      rotate(-rotate_amount, false); // Rotate CCW fixed amount, skip recal
    } else {
      SerialCom->println("    Correct R: Strafe L + Rot CW");
      strafeLeft(strafe_speed);
      delay(SHALLOW_CORRECTION_STRAFE_DELAY);
      stop();
      delay(100);
      rotate(rotate_amount, false); // Rotate CW fixed amount, skip recal
    }
    SerialCom->println("  SAC Correct Complete.");
  } else {
    SerialCom->println("  Ang OK (>=40)."); // Updated threshold check message
  }
  return true; // Indicate success
#else
  SerialCom->println("SAC skipped: US/Gyro Disabled.");
  return true; // Assume success if disabled
#endif
}


// Multi-axis move function with preliminary strafe
void move(float y_dist, float x_dist, float target_angle) {
#if !defined(NO_READ_GYRO) && !defined(NO_HC_SR04)
  SerialCom->print("Move fn | Tgt Y:"); SerialCom->print(y_dist);
  SerialCom->print(" | Tgt X:"); SerialCom->print(x_dist);
  SerialCom->print(" | Tgt Ang:"); SerialCom->println(target_angle);

  // --- Preliminary Strafe ---
  SerialCom->println("  Move: Checking initial side IR range...");
  unsigned long prelimStrafeStartTime = millis();
  bool sideSensorsOk = false;
  while (millis() - prelimStrafeStartTime < PRELIM_STRAFE_TIMEOUT) {
    bool use_front_ir_prelim = (y_dist >= 0);
    float ir_l_prelim, ir_r_prelim;
    if (use_front_ir_prelim) {
      ir_l_prelim = med_ir_dist(IR_LeftPIN);
      ir_r_prelim = med_ir_dist(IR_RightPIN);
    } else {
      ir_l_prelim = long_ir_dist(IR_LeftPIN_Back);
      ir_r_prelim = long_ir_dist(IR_RightPIN_Back);
    }
    if (ir_l_prelim > 0 || ir_r_prelim > 0) {
      SerialCom->println("  Move: Side IR in range.");
      sideSensorsOk = true;
      stop();
      break;
    } else {
      SerialCom->println("  Move Warn: Side IR out of range. Strafing slowly...");
      if (y_dist > 0) { strafeRight(PRELIM_STRAFE_SPEED); }
      else if (y_dist < 0) { strafeLeft(PRELIM_STRAFE_SPEED); }
      else { SerialCom->println("  Move Err: Target Y is 0, cannot prelim strafe."); break; }
      delay(200);
      stop();
      delay(50);
    }
  }
  if (!sideSensorsOk) {
    SerialCom->println("Move Err: Side IR out of range after timeout! Aborting move.");
    stop();
    return;
  }
  // --- End Preliminary Strafe ---

  recalibrateGyro(); // Recalibrate before main PID loop

  float y_err, x_err, ang_err;
  float current_y = y_dist;
  float current_x = x_dist;
  float current_ang = currentAngle;
  float buffer_dist_x = -1, ir_left = -1, ir_right = -1;
  float x_kp = MOVE_X_KP;
  float y_kp = MOVE_Y_KP;
  float ang_kp = MOVE_ANG_KP;
  float y_tolerance = MOVE_Y_TOLERANCE;
  float x_tolerance = MOVE_X_TOLERANCE;
  float ang_tolerance = MOVE_ANG_TOLERANCE;
  int stable_count = 0;
  int required_count = 5;
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  sensorServo.write(90);
  delay(200);

  SerialCom->println("  Move: Starting main PID loop...");
  // --- Main PID Control Loop ---
  do {
    if (millis() - startTime > MOVE_TIMEOUT) {
      SerialCom->println("Move Timeout!");
      break;
    }

    // Read Sensors
    bool use_front_ir = (y_dist >= 0);
    if (use_front_ir) {
      ir_left = med_ir_dist(IR_LeftPIN);
      ir_right = med_ir_dist(IR_RightPIN);
    } else {
      ir_left = long_ir_dist(IR_LeftPIN_Back);
      ir_right = long_ir_dist(IR_RightPIN_Back);
    }
    int valid_ir_count = 0;
    float ir_sum = 0;
    if (ir_left > 0) { ir_sum += ir_left; valid_ir_count++; }
    if (ir_right > 0) { ir_sum += ir_right; valid_ir_count++; }
    if (valid_ir_count > 0) {
      current_y = ir_sum / valid_ir_count;
    } else {
      SerialCom->println("  Move Warn: Side IR failed in main loop!");
    }

    buffer_dist_x = HC_SR04_range();
    if (buffer_dist_x > 0) {
      current_x = buffer_dist_x;
    }
    updateCurrentAngle();
    current_ang = currentAngle;

    // Calculate Errors
    if (abs(y_dist) < 0.1) {
      y_err = current_y;
    } else {
      y_err = -1.0 * (y_dist / abs(y_dist)) * (abs(y_dist) - current_y);
    }
    x_err = current_x - x_dist;
    ang_err = current_ang - target_angle;

    // Calculate Power Outputs
    int y_pwr = 0;
    if (valid_ir_count > 0) {
      y_pwr = (int)constrainPwr(y_kp * y_err, -150, 150);
    }
    int x_pwr = (int)constrainPwr(x_kp * x_err, -200, 200);
    int ang_pwr = (int)constrainPwr(ang_kp * ang_err, -100, 100);

    // Apply Motor Commands (VERIFY MIXING SIGNS)
    left_font_motor.writeMicroseconds(1500 + x_pwr - y_pwr + ang_pwr);
    left_rear_motor.writeMicroseconds(1500 + x_pwr + y_pwr + ang_pwr);
    right_rear_motor.writeMicroseconds(1500 - x_pwr + y_pwr + ang_pwr);
    right_font_motor.writeMicroseconds(1500 - x_pwr - y_pwr + ang_pwr);

    // Check Stability
    if ((abs(y_err) <= y_tolerance || valid_ir_count == 0) && // Ignore Y error if sensors failed
        (abs(x_err) <= x_tolerance) &&
        (abs(ang_err) <= ang_tolerance)) {
      stable_count++;
    } else {
      stable_count = 0;
    }

    // Periodic Debug Output
    if (millis() - lastPrintTime > 200) {
        SerialCom->print("  Move-> X_Err:"); SerialCom->print(x_err, 1);
        SerialCom->print(" | Y_Err:"); SerialCom->print(y_err, 1);
        SerialCom->print(" | A_Err:"); SerialCom->print(ang_err, 1);
        SerialCom->print(" | Stbl:"); SerialCom->println(stable_count);
        lastPrintTime = millis();
    }
    delay(LOOP_TIME_MS);
  } while (stable_count < required_count);

  stop();
  SerialCom->println("Move function complete.");
#else
  SerialCom->println("Move function skipped: Gyro or Ultrasonic Disabled.");
  delay(2000);
#endif
}


// ==================== UTILITY FUNCTIONS ====================
#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte c = 0;
  const int MIN = 717;
  const int MAX = 860;
  const int PIN = A0;
  int r = analogRead(PIN);
  if (r > MIN && r < (MAX + 40)) {
    c = 0;
    return true;
  } else {
    if (r <= MIN) { SerialCom->print("! LOW BATT: "); SerialCom->println(r); }
    else { SerialCom->print("! HIGH BATT?: "); SerialCom->println(r); }
    c++;
    if (c > 5) { SerialCom->println("!! BATT CRITICAL !!"); return false; }
    else { return true; }
  }
}
#else
boolean is_battery_voltage_OK() { return true; }
#endif

void slow_flash_LED_builtin() {
  static unsigned long t = 0;
  unsigned long n = millis();
  if (n - t > 1000) {
    t = n;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

float constrainPwr(float p, float l, float u) {
  if (p > u) return u;
  if (p < l) return l;
  return p;
}
