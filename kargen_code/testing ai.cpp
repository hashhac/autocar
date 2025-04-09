/*
  Refactored MechEng 706 Base Code
  Focus: Find a wall and align perpendicular to it.

  Based on original code by Logan Stuart, refactored for clarity
  and specific task focus. Incorporates calibrated strafe.
*/
#include <Servo.h>  // Need for Servo pulse output
#include <Arduino.h> // Include standard Arduino functions

// --- Configuration ---
// #define NO_READ_GYRO      // Uncomment if GYRO is not attached.
// #define NO_HC_SR04        // Uncomment if HC-SR04 ultrasonic is not attached.
// #define NO_BATTERY_V_OK   // Uncomment if you do not care about battery voltage/damage.

// State machine states
enum STATE {
  INITIALISING,
  RUNNING, // Kept for structure, but test sequence goes to STOPPED
  STOPPED,
  TEST     // The main autonomous sequence state
};

bool testing = true; // Set to true to run the TEST state sequence

// --- Pin Definitions ---
// Refer to Shield Pinouts.jpg for pin locations

// Motor control pins (Vex Motor Controller 29 via Servo)
const byte left_front_pin = 46;
const byte left_rear_pin = 47;
const byte right_rear_pin = 50;
const byte right_front_pin = 51;

// Ultrasonic ranging sensor pins (HC-SR04)
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// IR Sensor Pins (Sharp GP2Y0A series)
const int IR_LeftPIN = A4;       // Medium Range Front Left
const int IR_RightPIN = A5;      // Medium Range Front Right
const int IR_LeftPIN_Back = A6;  // Long Range Back Left (Check pin A6 exists/is correct)
const int IR_RightPIN_Back = A7; // Long Range Back Right (Check pin A7 exists/is correct)

// Gyro Sensor Pin (MPU-9250 - assuming analog Z-axis output)
const int gyroPin = A3;

// Servo Pin for Ultrasonic Sensor Turret
const int SERVO_PIN = 7; // Use a descriptive name

// --- Constants ---
// Ultrasonic Sensor
const unsigned int MAX_DIST_US = 23200; // Max pulse width for HC-SR04 (~400 cm)
const unsigned long US_TIMEOUT = MAX_DIST_US + 1000; // Timeout for pulseIn

// IR Sensor Calibration (Adjust these based on your specific sensors and calibration)
// Medium Range (e.g., GP2Y0A41SK0F, 4-30cm) - EXAMPLE VALUES
const float MED_IR_MULT = 2118.6; // Example: Constant A for medium range
const float MED_IR_OFFSET = 20.072;// Example: Constant B for medium range
// Long Range (e.g., GP2Y0A21YK0F, 10-80cm) - EXAMPLE VALUES
const float LONG_IR_MULT = 4261.4; // Example: Constant A for long range
const float LONG_IR_OFFSET = 61.06; // Example: Constant B for long range
const int IR_MIN_READING = 110;    // Minimum plausible raw ADC reading
const int IR_MAX_READING = 550;    // Maximum plausible raw ADC reading

// Gyro Sensor Calibration (Adjust these based on your specific sensor)
const float gyroSupplyVoltage = 5.0; // Arduino supply voltage
float gyroZeroVoltage = 1.65;        // Default zero-rate voltage (MUST RECALIBRATE)
const float gyroSensitivity = 0.007; // Sensitivity in V/(deg/s) (e.g., 7mV per deg/s)
const float rotationThreshold = 1.5; // Ignore gyro rates below this (deg/s) to reduce drift integration

// Movement
const int MAX_SPEED = 300; // Maximum speed value for motors (adjust as needed)
int speed_val = 150;       // Default speed for basic movements (can be overridden)
const unsigned long TIMEOUT_DURATION = 10000; // Max duration for timed movements (ms)
const int LOOP_TIME_MS = 50; // Target loop time for gyro updates etc. (T)

// Strafe Calibration Factors (Adjust these based on testing your robot)
const float strafe_motor_TL_speed = 1.0;    // Top-Left
const float strafe_motor_TR_speed = 0.993;  // Top-Right
const float strafe_motor_BL_speed = 0.993;  // Bottom-Left
const float strafe_motor_BR_speed = 1.0;    // Bottom-Right

// --- Global Variables ---
// Motor Servo Objects
Servo left_font_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_font_motor;

// Ultrasonic Turret Servo Object
Servo sensorServo; // Renamed from myservo

// Gyro Variables
float currentAngle = 0.0; // Robot's current angle relative to starting orientation

// Serial Pointer (for easy switching between Serial and Serial1/Bluetooth)
HardwareSerial* SerialCom;

// --- Function Declarations ---
// State Machine
STATE initialising();
STATE running();
STATE stopped();
STATE test();

// Sensor Functions
float HC_SR04_range();
float med_ir_dist(int ir_pin);
float long_ir_dist(int ir_pin);
void recalibrateGyro();
void updateCurrentAngle();
bool checkAngle(float targetAngle);
#ifndef NO_READ_GYRO
void GYRO_reading(); // Keep for potential debugging
#endif
void Analog_Range_A4(); // Keep for potential debugging

// Movement Functions
void enable_motors();
void disable_motors();
void stop();
void forward(int speed = speed_val); // Default speed parameter
void reverse(int speed = speed_val);
void ccw(int speed = speed_val);
void cw(int speed = speed_val);
void strafeLeft(int speed = speed_val); // Use calibrated version
void strafeRight(int speed = speed_val);// Use calibrated version
void rotate(int degrees);
void forwardUntil(int targetDistance, float kp);
void orientate();
void move(float y_dist, float x_dist, float target_angle); // Keep complex move if needed later

// Utility Functions
boolean is_battery_voltage_OK();
void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
float constrainPwr(float pwr); // Overload 1
float constrainPwr(float pwr, float lower, float upper); // Overload 2
void read_serial_command(); // Keep for manual testing if needed
void speed_change_smooth(); // Keep for manual testing if needed


// ==================== SETUP ====================
void setup(void) {
  // Initialize Serial Communication
  SerialCom = &Serial; // Default to USB Serial
  SerialCom->begin(115200);
  while (!Serial); // Wait for Serial port to connect (especially for Leonardo/Micro)
  SerialCom->println("\n=== Robot Initializing ===");
  SerialCom->println("Code: Refactored Find and Align Perpendicular");

  // Built-in LED for status
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Attach Ultrasonic Turret Servo
  sensorServo.attach(SERVO_PIN);
  sensorServo.write(90); // Point sensor forward initially
  SerialCom->println("Ultrasonic servo attached and centered.");

  // Initialize Gyro
  pinMode(gyroPin, INPUT);
  SerialCom->println("Gyro pin set to input.");
  // IMPORTANT: Initial gyro calibration happens in the first rotate/move call

  // Initialize Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  SerialCom->println("Ultrasonic pins initialized.");

  // Initialize IR Sensor Pins (implicitly INPUT_ANALOG)
  SerialCom->println("IR sensor pins ready.");

  // Motors are enabled in initialising() state

  SerialCom->println("Setup complete. Waiting 1 second...");
  delay(1000);
}

// ==================== MAIN LOOP ====================
void loop(void) {
  static STATE machine_state = INITIALISING; // Start in INITIALISING state

  // Finite-state machine execution
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: // This state is now less relevant if only running TEST
      machine_state = running();
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
  digitalWrite(LED_BUILTIN, HIGH); // LED ON during init
  SerialCom->println("Enabling Motors...");
  enable_motors(); // Attach servos to pins
  stop();          // Ensure motors are stopped initially
  SerialCom->println("Motors enabled and stopped.");

#ifndef NO_READ_GYRO
  SerialCom->println("Performing initial Gyro calibration...");
  recalibrateGyro(); // Calibrate gyro once at the start
#endif

  digitalWrite(LED_BUILTIN, LOW); // LED OFF after init

  if (testing) {
    SerialCom->println("Transitioning to TEST state.");
    return TEST; // Go directly to the autonomous sequence
  } else {
    SerialCom->println("Transitioning to RUNNING state (manual control).");
    return RUNNING; // Go to manual control state
  }
}

STATE running() {
  // This state is for manual control via serial commands
  // Or other non-test operations.
  static unsigned long previous_millis = 0;

  // Check for serial commands
  read_serial_command();

  // Flash LED to indicate running state
  fast_flash_double_LED_builtin();

  // Optional: Periodic sensor readings or status updates
  if (millis() - previous_millis > 1000) { // Print status every second
    previous_millis = millis();
    SerialCom->println("--- STATE: RUNNING (Manual Mode) ---");

    #ifndef NO_HC_SR04
    float dist = HC_SR04_range();
    SerialCom->print("Ultrasonic Dist: "); SerialCom->println(dist > 0 ? String(dist) : "Error/OOR");
    #endif

    #ifndef NO_READ_GYRO
    SerialCom->print("Current Angle: "); SerialCom->println(currentAngle);
    #endif

    #ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) {
       SerialCom->println("Battery low! Transitioning to STOPPED state.");
       return STOPPED; // Go to stopped state if battery is low
    }
    #endif
  }

  return RUNNING; // Stay in running state
}

STATE test() {
  // --- Autonomous Sequence: Find Wall and Align Perpendicular ---
  float driving_until_dist = 20.0; // Target distance to stop in front of wall (cm)
  int approach_kp = 8;           // Proportional gain for forwardUntil
  int rotation_angle = 90;       // Rotation angle to become perpendicular

  SerialCom->println("\n=== STATE: TEST - Find and Align Perpendicular ===");
  sensorServo.write(90); // Ensure sensor is facing forward
  delay(500);

  // --- Step 1: Approach the nearest wall ---
  SerialCom->println("--- Step 1: Approaching wall ---");
  // Measure initial distance to avoid immediate movement if already close
  float initial_dist_avg = 0;
  int valid_readings = 0;
  for (int i = 0; i < 3; i++) {
      float dist = HC_SR04_range();
      if (dist > 0) {
          initial_dist_avg += dist;
          valid_readings++;
      }
      delay(50); // Short delay between pings
  }

  if (valid_readings > 0) {
      initial_dist_avg /= (float)valid_readings;
      SerialCom->print("Initial average distance: "); SerialCom->print(initial_dist_avg); SerialCom->println(" cm");

      if (initial_dist_avg > driving_until_dist) {
          SerialCom->println("Moving forward to wall...");
          forwardUntil(driving_until_dist, approach_kp);
          SerialCom->println("Approach complete.");
      } else {
          SerialCom->println("Already close to wall, skipping approach.");
      }
  } else {
      SerialCom->println("Could not get initial distance reading. Stopping.");
      return STOPPED; // Stop if sensor fails
  }
  delay(1000); // Pause after approaching

  // --- Step 2: Orient parallel to the wall using IR sensors ---
  SerialCom->println("--- Step 2: Orienting parallel to the wall ---");
  orientate(); // Uses front IR sensors
  SerialCom->println("Orientation parallel complete.");
  delay(1000); // Pause after orienting

  // --- Step 3: Rotate 90 degrees to become perpendicular ---
  SerialCom->println("--- Step 3: Rotating to perpendicular ---");
  SerialCom->print("Rotating "); SerialCom->print(rotation_angle); SerialCom->println(" degrees...");
  rotate(rotation_angle); // Use gyro to rotate
  SerialCom->println("Rotation perpendicular complete.");
  delay(1000); // Pause after rotating

  SerialCom->println("=== TEST SEQUENCE COMPLETE ===");
  SerialCom->println("Transitioning to STOPPED state.");
  return STOPPED; // End the test sequence and stop
}


STATE stopped() {
  // State entered when battery is low or task is complete
  static unsigned long previous_millis = 0;
  static byte counter_lipo_voltage_ok = 0;

  disable_motors(); // Ensure motors are off
  slow_flash_LED_builtin(); // Visual indicator

  // Print status periodically
  if (millis() - previous_millis > 1000) {
    previous_millis = millis();
    SerialCom->println("--- STATE: STOPPED ---");

    #ifndef NO_BATTERY_V_OK
    // Check if battery has recovered (only relevant if stopped due to low battery)
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK, waiting for stable voltage (count: ");
      SerialCom->print(counter_lipo_voltage_ok); SerialCom->println("/10)");
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { // Wait for 10s of stable voltage
        counter_lipo_voltage_ok = 0;
        SerialCom->println("Battery voltage stable. Transitioning to RUNNING.");
        // Decide whether to re-run TEST or go to RUNNING
        // For now, go to RUNNING for manual control after recovery.
        // If you want to retry TEST, change 'return RUNNING' to 'return TEST'
        // after enabling motors.
        enable_motors(); // Re-enable motors before leaving stopped state
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0; // Reset counter if voltage drops again
      SerialCom->println("Battery voltage still low or task complete.");
    }
    #else
    SerialCom->println("Robot stopped (Task complete or manual stop).");
    #endif
  }
  return STOPPED; // Stay stopped
}


// ==================== SENSOR FUNCTIONS ====================

#ifndef NO_HC_SR04
// Reads distance using HC-SR04 Ultrasonic sensor
float HC_SR04_range() {
  unsigned long duration;
  float cm;

  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse duration with timeout
  duration = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);

  // Calculate distance
  if (duration > 0 && duration < MAX_DIST_US) {
    cm = duration / 58.0;
    // SerialCom->print("HC-SR04: "); SerialCom->print(cm); SerialCom->println(" cm"); // Optional: Print every reading
    return cm;
  } else if (duration == 0) {
    // SerialCom->println("HC-SR04: Timeout (No Echo)"); // Error message
    return -1.0; // Indicate timeout error
  } else {
    // SerialCom->println("HC-SR04: Out of range"); // Error message
    return -2.0; // Indicate out of range error (pulse too long)
  }
}
#else
// Dummy function if sensor is disabled
float HC_SR04_range() {
  SerialCom->println("HC-SR04 Disabled");
  return -1.0;
}
#endif

// Reads distance from Medium Range IR sensor (e.g., 4-30cm)
float med_ir_dist(int ir_pin) {
  int analog_value = analogRead(ir_pin);
  float distance = -1.0; // Default to error

  if ((analog_value >= IR_MIN_READING) && (analog_value <= IR_MAX_READING)) {
    // Apply calibration formula (ensure no division by zero or near-zero)
    if (abs(analog_value - MED_IR_OFFSET) > 0.1) { // Check denominator
       distance = MED_IR_MULT / (analog_value - MED_IR_OFFSET);
       // Optional: Clamp distance to expected sensor range (e.g., 4-30cm)
       // distance = constrain(distance, 4.0, 30.0);
    } else {
        // SerialCom->print("Med IR Error: Denominator near zero (Pin "); SerialCom->print(ir_pin); SerialCom->println(")");
        distance = -2.0; // Indicate calculation error
    }
  } else {
     // SerialCom->print("Med IR Reading Out of Plausible Range: "); SerialCom->println(analog_value);
     distance = -3.0; // Indicate reading out of range
  }
  // SerialCom->print("Med IR (Pin "); SerialCom->print(ir_pin); SerialCom->print("): "); SerialCom->println(distance > 0 ? String(distance) : "Error");
  return distance;
}

// Reads distance from Long Range IR sensor (e.g., 10-80cm)
float long_ir_dist(int ir_pin) {
  int analog_value = analogRead(ir_pin);
  float distance = -1.0; // Default to error

  if ((analog_value >= IR_MIN_READING) && (analog_value <= IR_MAX_READING)) {
     if (abs(analog_value - LONG_IR_OFFSET) > 0.1) { // Check denominator
       distance = LONG_IR_MULT / (analog_value - LONG_IR_OFFSET);
       // Optional: Clamp distance to expected sensor range (e.g., 10-80cm)
       // distance = constrain(distance, 10.0, 80.0);
     } else {
        // SerialCom->print("Long IR Error: Denominator near zero (Pin "); SerialCom->print(ir_pin); SerialCom->println(")");
        distance = -2.0; // Indicate calculation error
     }
  } else {
     // SerialCom->print("Long IR Reading Out of Plausible Range: "); SerialCom->println(analog_value);
     distance = -3.0; // Indicate reading out of range
  }
  // SerialCom->print("Long IR (Pin "); SerialCom->print(ir_pin); SerialCom->print("): "); SerialCom->println(distance > 0 ? String(distance) : "Error");
  return distance;
}


#ifndef NO_READ_GYRO
// Recalibrates the gyro to find the zero-rate voltage
void recalibrateGyro() {
  SerialCom->println("Recalibrating gyroscope...");
  long sum = 0; // Use long for sum to avoid overflow
  const int numReadings = 200; // Increase readings for better accuracy

  // Ensure motors are stopped during calibration
  stop();
  delay(500); // Allow time for robot to stabilize

  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(gyroPin); // Read raw gyro value
    delay(5);                   // Small delay between readings
  }

  // Calculate the new zero voltage
  gyroZeroVoltage = ((float)sum / numReadings) * (gyroSupplyVoltage / 1023.0); // Convert ADC to voltage

  SerialCom->print("Gyro calibration complete. New Zero Voltage: ");
  SerialCom->println(gyroZeroVoltage, 4); // Print with more precision
  currentAngle = 0.0; // Reset angle after calibration
  delay(100); // Short pause after calibration
}

// Updates the current angle estimate using the gyroscope
void updateCurrentAngle() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0; // Time difference in seconds

  if (dt <= 0) return; // Avoid division by zero if called too quickly

  lastUpdateTime = currentTime;

  int rawValue = analogRead(gyroPin);
  // Calculate voltage, subtract zero offset, convert to angular velocity (deg/s)
  float gyroRate = (((rawValue * gyroSupplyVoltage) / 1023.0) - gyroZeroVoltage) / gyroSensitivity;

  // Integrate only if rate is above the threshold to reduce drift
  if (abs(gyroRate) >= rotationThreshold) {
    currentAngle += gyroRate * dt; // Integrate angular velocity over time
  }

  // Optional: Wrap angle (e.g., to +/- 180 or 0-360) if needed
  // currentAngle = fmod(currentAngle, 360.0);
  // if (currentAngle < 0) currentAngle += 360.0;

  // SerialCom->print("Raw Gyro: "); SerialCom->print(rawValue);
  // SerialCom->print(" | Rate: "); SerialCom->print(gyroRate);
  // SerialCom->print(" | Angle: "); SerialCom->println(currentAngle);
}


// Checks if the robot has reached the target angle within tolerance
bool checkAngle(float targetAngle) {
  // Ensure the angle is updated frequently before checking
  // Consider calling updateCurrentAngle() within the loop that calls checkAngle()
  float angleDifference = abs(targetAngle - currentAngle);

  // SerialCom->print("Checking Angle -> Target: "); SerialCom->print(targetAngle);
  // SerialCom->print(" | Current: "); SerialCom->print(currentAngle);
  // SerialCom->print(" | Diff: "); SerialCom->println(angleDifference);

  return angleDifference <= rotationThreshold; // Use rotationThreshold as tolerance
}

// Prints the raw gyro reading (for debugging)
void GYRO_reading() {
  SerialCom->print("Raw GYRO A3 Reading: ");
  SerialCom->println(analogRead(gyroPin));
}

#else
// Dummy functions if gyro is disabled
void recalibrateGyro() { SerialCom->println("Gyro Disabled - Recalibration Skipped"); }
void updateCurrentAngle() { /* Gyro Disabled */ }
bool checkAngle(float targetAngle) { SerialCom->println("Gyro Disabled - checkAngle always true"); return true; }
void GYRO_reading() { SerialCom->println("Gyro Disabled"); }
#endif

// Prints raw reading of an analog pin (for debugging)
void Analog_Range_A4() {
  SerialCom->print("Raw Analog A4 Reading: ");
  SerialCom->println(analogRead(A4)); // Note: A4 is IR_LeftPIN
}


// ==================== MOVEMENT FUNCTIONS ====================

// Attaches servos to pins, enabling motor control
void enable_motors() {
  // Ensure pins are OUTPUT before attaching
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

// Detaches servos, disabling motor control (saves power, stops signals)
void disable_motors() {
  left_font_motor.detach();
  left_rear_motor.detach();
  right_rear_motor.detach();
  right_font_motor.detach();

  // Optional: Set pins to INPUT to prevent floating
  pinMode(left_front_pin, INPUT);
  pinMode(left_rear_pin, INPUT);
  pinMode(right_rear_pin, INPUT);
  pinMode(right_front_pin, INPUT);
  SerialCom->println("Motors Disabled.");
}

// Stops all motors (sends neutral signal)
void stop() {
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
  // SerialCom->println("Motors Stopped."); // Can be too verbose
}

// Moves robot forward
void forward(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 - speed);
}

// Moves robot backward
void reverse(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 + speed);
}

// Rotates robot counter-clockwise (原地左转)
void ccw(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 - speed);
  left_rear_motor.writeMicroseconds(1500 - speed);
  right_rear_motor.writeMicroseconds(1500 - speed);
  right_font_motor.writeMicroseconds(1500 - speed);
}

// Rotates robot clockwise (原地右转)
void cw(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  left_font_motor.writeMicroseconds(1500 + speed);
  left_rear_motor.writeMicroseconds(1500 + speed);
  right_rear_motor.writeMicroseconds(1500 + speed);
  right_font_motor.writeMicroseconds(1500 + speed);
}

// Strafes left using calibrated speeds
void strafeLeft(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  // Apply calibration factors to target speed for each wheel
  left_font_motor.writeMicroseconds(1500 - (int)(speed * strafe_motor_TL_speed));
  left_rear_motor.writeMicroseconds(1500 + (int)(speed * strafe_motor_BL_speed));
  right_rear_motor.writeMicroseconds(1500 + (int)(speed * strafe_motor_BR_speed));
  right_font_motor.writeMicroseconds(1500 - (int)(speed * strafe_motor_TR_speed));
  // SerialCom->print("Strafing left, speed: "); SerialCom->println(speed);
}

// Strafes right using calibrated speeds
void strafeRight(int speed) {
  speed = constrain(speed, 0, MAX_SPEED);
  // Apply calibration factors to target speed for each wheel
  left_font_motor.writeMicroseconds(1500 + (int)(speed * strafe_motor_TL_speed));
  left_rear_motor.writeMicroseconds(1500 - (int)(speed * strafe_motor_BL_speed));
  right_rear_motor.writeMicroseconds(1500 - (int)(speed * strafe_motor_BR_speed));
  right_font_motor.writeMicroseconds(1500 + (int)(speed * strafe_motor_TR_speed));
  // SerialCom->print("Strafing right, speed: "); SerialCom->println(speed);
}


// Rotates the robot by a specific number of degrees using the gyro
void rotate(int degrees) {
#ifndef NO_READ_GYRO
  SerialCom->print("Rotate function called. Target degrees: "); SerialCom->println(degrees);
  recalibrateGyro(); // Recalibrate before each rotation for best accuracy

  float targetAngle = currentAngle + degrees; // Calculate absolute target angle
  int kp = 10; // Proportional gain for rotation speed
  float motor_pwr;
  float error;
  int rotation_speed;
  unsigned long startTime = millis();

  SerialCom->print("Starting angle: "); SerialCom->println(currentAngle);
  SerialCom->print("Target angle: "); SerialCom->println(targetAngle);

  do {
    // Check for timeout
    if (millis() - startTime > TIMEOUT_DURATION) {
        SerialCom->println("Rotate Timeout! Stopping.");
        break;
    }

    updateCurrentAngle(); // Update angle reading
    error = targetAngle - currentAngle;

    // Calculate motor power based on error
    motor_pwr = kp * error;
    rotation_speed = (int)constrainPwr(abs(motor_pwr), 50, MAX_SPEED); // Apply power constraints (min speed 50)

    // Determine rotation direction and execute
    if (error > 0) {
      // Need to rotate CW
      cw(rotation_speed);
      // SerialCom->print("Rotating CW, Speed: "); SerialCom->println(rotation_speed);
    } else {
      // Need to rotate CCW
      ccw(rotation_speed);
      // SerialCom->print("Rotating CCW, Speed: "); SerialCom->println(rotation_speed);
    }

    delay(LOOP_TIME_MS / 2); // Short delay within loop

  } while (!checkAngle(targetAngle)); // Loop until target angle is reached

  stop(); // Stop the robot after reaching the target angle
  SerialCom->print("Rotation complete. Final angle: "); SerialCom->println(currentAngle);
#else
  SerialCom->println("Rotate function skipped: Gyro Disabled.");
  delay(abs(degrees) * 10); // Simple delay if gyro disabled
#endif
}


// Moves forward/backward until a target distance is reached using Ultrasonic
void forwardUntil(int targetDistance, float kp) {
#ifndef NO_HC_SR04
  SerialCom->print("forwardUntil called. Target distance: "); SerialCom->print(targetDistance); SerialCom->println(" cm");
  float current_distance = 100.0; // Assume initially far
  float buffer_dist = -1.0;
  float error;
  int move_speed;
  int stable_count = 0;
  int required_count = 5; // Need 5 consecutive readings within tolerance
  float tolerance = 1.0;  // +/- 1 cm tolerance
  unsigned long startTime = millis();

  sensorServo.write(90); // Ensure sensor faces forward
  delay(200); // Allow servo to settle

  do {
    // Check for timeout
    if (millis() - startTime > TIMEOUT_DURATION) {
      SerialCom->println("forwardUntil Timeout! Stopping.");
      break;
    }

    // Get distance reading
    buffer_dist = HC_SR04_range();
    if (buffer_dist > 0) { // Only update if reading is valid
      current_distance = buffer_dist;
      // SerialCom->print("Current Distance: "); SerialCom->println(current_distance);
    } else {
      SerialCom->print("forwardUntil Warning: Invalid US reading ("); SerialCom->print(buffer_dist); SerialCom->println("). Using previous distance.");
      // Keep using the last known valid distance if reading fails
    }

    // Calculate error and motor speed
    error = current_distance - targetDistance;
    move_speed = (int)constrainPwr(abs(error * kp), 50, MAX_SPEED); // Min speed 50

    // Check if target reached
    if (abs(error) <= tolerance) {
      stable_count++;
      // SerialCom->print("Within tolerance. Count: "); SerialCom->println(stable_count);
      stop(); // Stop briefly when within tolerance
      delay(LOOP_TIME_MS); // Small pause
    } else {
      stable_count = 0; // Reset counter if outside tolerance
      // Move based on error sign
      if (error > 0) {
        forward(move_speed); // Move forward if too far
      } else {
        reverse(move_speed); // Move backward if too close
      }
    }

    delay(LOOP_TIME_MS); // Loop delay

  } while (stable_count < required_count);

  stop(); // Stop the robot once target distance is stable
  SerialCom->print("Target Distance Reached and Stable: "); SerialCom->println(current_distance);
#else
  SerialCom->println("forwardUntil skipped: Ultrasonic Disabled.");
  delay(1000); // Simple delay if sensor disabled
#endif
}


// Orients the robot parallel to a wall in front using front IR sensors
void orientate() {
  SerialCom->println("Orientate function called...");
  float tolerance = 1.0; // Increased tolerance for IR sensors (cm)
  float ir_left, ir_right;
  int rotation_speed = 75; // Slow speed for fine adjustment
  int stable_count = 0;
  int required_count = 5; // Need 5 consecutive readings within tolerance
  unsigned long startTime = millis();
  float error;

  do {
    // Check for timeout
    if (millis() - startTime > TIMEOUT_DURATION / 2) { // Shorter timeout for orientation
      SerialCom->println("Orientate Timeout! Stopping.");
      break;
    }

    // Read IR sensors
    ir_left = med_ir_dist(IR_LeftPIN);
    ir_right = med_ir_dist(IR_RightPIN);

    SerialCom->print("Orientate -> Left IR: "); SerialCom->print(ir_left > 0 ? String(ir_left) : "Err");
    SerialCom->print(" | Right IR: "); SerialCom->println(ir_right > 0 ? String(ir_right) : "Err");

    // Check if both readings are valid
    if (ir_left > 0 && ir_right > 0) {
      error = ir_left - ir_right; // Calculate difference

      // Check if within tolerance
      if (abs(error) < tolerance) {
        stable_count++;
        // SerialCom->print("Orientate: Within tolerance. Count: "); SerialCom->println(stable_count);
        stop(); // Stop briefly
        delay(LOOP_TIME_MS);
      } else {
        stable_count = 0; // Reset counter
        // Rotate based on which sensor is further away
        if (error > 0) { // Left is further, rotate CCW
          ccw(rotation_speed);
        } else { // Right is further, rotate CW
          cw(rotation_speed);
        }
      }
    } else {
      // Handle invalid sensor readings - stop and report error
      SerialCom->println("Orientate Error: Invalid IR reading(s). Stopping.");
      stop();
      stable_count = 0; // Reset count on error
      delay(200); // Wait before potentially retrying
    }

    delay(LOOP_TIME_MS); // Loop delay

  } while (stable_count < required_count);

  stop(); // Ensure robot is stopped
  SerialCom->println("Orientation Parallel Complete.");
}


// Complex move function (Kept for potential future use, but not used in current TEST)
void move(float y_dist, float x_dist, float target_angle) {
#if !defined(NO_READ_GYRO) && !defined(NO_HC_SR04)
  SerialCom->print("Move function called. Target Y:"); SerialCom->print(y_dist);
  SerialCom->print(" | Target X:"); SerialCom->print(x_dist);
  SerialCom->print(" | Target Ang:"); SerialCom->println(target_angle);

  recalibrateGyro(); // Recalibrate before complex move

  float y_err, x_err, ang_err;
  float current_y = y_dist; // Assume starting at target initially
  float current_x = x_dist; // Assume starting at target initially
  float current_ang = currentAngle;
  float buffer_dist_x = -1, ir_left = -1, ir_right = -1;

  int y_pwr, x_pwr, ang_pwr;
  float x_kp = 40, y_kp = 20, ang_kp = 5; // Adjusted gains (tune these!)
  float y_tolerance = 1.5, x_tolerance = 1.5, ang_tolerance = 2.0; // Adjusted tolerances
  int stable_count = 0;
  int required_count = 5;
  unsigned long startTime = millis();

  do {
    // Timeout Check
     if (millis() - startTime > TIMEOUT_DURATION * 2) { // Longer timeout for move
      SerialCom->println("Move Timeout! Stopping.");
      break;
    }

    // --- Sensor Readings ---
    // Y Distance (IR Sensors - Side)
    bool use_front_ir = (y_dist > 0); // Example logic: positive y = front sensors
    if (use_front_ir) {
      ir_left = med_ir_dist(IR_LeftPIN);
      ir_right = med_ir_dist(IR_RightPIN);
    } else {
      ir_left = long_ir_dist(IR_LeftPIN_Back); // Assumes back sensors for negative Y
      ir_right = long_ir_dist(IR_RightPIN_Back);
    }

    // Average valid IR readings for Y distance
    if (ir_left > 0 && ir_right > 0) {
      current_y = (ir_left + ir_right) / 2.0;
    } else if (ir_left > 0) {
      current_y = ir_left; // Use single valid reading
    } else if (ir_right > 0) {
      current_y = ir_right; // Use single valid reading
    } // else: keep previous current_y if both fail

    // X Distance (Ultrasonic - Front)
    buffer_dist_x = HC_SR04_range();
    if (buffer_dist_x > 0) {
      current_x = buffer_dist_x;
    } // else: keep previous current_x if reading fails

    // Angle (Gyro)
    updateCurrentAngle();
    current_ang = currentAngle;

    // --- Error Calculation ---
    // Y Error: Difference from target side distance
    y_err = current_y - abs(y_dist); // Error is difference from target magnitude
    if (y_dist < 0) y_err *= -1; // Invert error sign if target is negative Y

    // X Error: Difference from target front distance
    x_err = current_x - x_dist;

    // Angle Error: Difference from target angle
    ang_err = target_angle - current_ang; // Simple difference

    // --- PID Control (Proportional Only) ---
    // Calculate power for each axis, constrain values
    // Note: Signs depend on how error relates to motor commands
    // This requires careful tuning and understanding of robot kinematics
    y_pwr = (int)constrainPwr(y_kp * y_err, -150, 150); // Strafe power
    x_pwr = (int)constrainPwr(x_kp * x_err, -200, 200); // Forward/Reverse power
    ang_pwr = (int)constrainPwr(ang_kp * ang_err, -100, 100); // Rotation power

    // --- Motor Commands (Combine axis powers) ---
    // This mixing logic assumes standard Mecanum setup. Adjust if needed.
    // Positive y_pwr = strafe right? Positive x_pwr = forward? Positive ang_pwr = cw? CHECK THIS!
    // Example mixing (adjust signs based on testing):
    left_font_motor.writeMicroseconds(1500 + x_pwr + y_pwr + ang_pwr);
    left_rear_motor.writeMicroseconds(1500 + x_pwr - y_pwr + ang_pwr);
    right_rear_motor.writeMicroseconds(1500 - x_pwr - y_pwr + ang_pwr);
    right_font_motor.writeMicroseconds(1500 - x_pwr + y_pwr + ang_pwr);


    // --- Check Stability ---
    if ((abs(y_err) <= y_tolerance) && (abs(x_err) <= x_tolerance) && (abs(ang_err) <= ang_tolerance)) {
      stable_count++;
    } else {
      stable_count = 0;
    }

    // --- Debugging Output ---
    SerialCom->print("Move -> X Err: "); SerialCom->print(x_err);
    SerialCom->print(" | Y Err: "); SerialCom->print(y_err);
    SerialCom->print(" | Ang Err: "); SerialCom->print(ang_err);
    SerialCom->print(" | Stable: "); SerialCom->println(stable_count);

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
// Checks Lipo battery voltage level
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter = 0;
  const int MIN_RAW_LIPO = 717; // Raw ADC value corresponding to ~3.5V per cell (7V total for 2S)
  const int MAX_RAW_LIPO = 860; // Raw ADC value corresponding to ~4.2V per cell (8.4V total for 2S)
  const int VOLTAGE_PIN = A0; // Pin for battery voltage divider

  int raw_lipo = analogRead(VOLTAGE_PIN);

  // Basic check if voltage is within a plausible operating range
  if (raw_lipo > MIN_RAW_LIPO && raw_lipo < (MAX_RAW_LIPO + 40)) { // Allow slight overcharge reading
    Low_voltage_counter = 0; // Reset counter if voltage is OK
    // Optional: Calculate and print percentage
    // int Lipo_level_cal = map(raw_lipo, MIN_RAW_LIPO, MAX_RAW_LIPO, 0, 100);
    // SerialCom->print("Lipo level: "); SerialCom->print(constrain(Lipo_level_cal, 0, 110)); SerialCom->println("%");
    return true;
  } else {
    // Voltage is too low or potentially disconnected/overcharged
    if (raw_lipo <= MIN_RAW_LIPO) {
       SerialCom->print("!!! WARNING: Lipo voltage LOW: "); SerialCom->println(raw_lipo);
    } else {
       SerialCom->print("!!! WARNING: Lipo voltage reading HIGH (Overcharged?): "); SerialCom->println(raw_lipo);
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5) { // Require 5 consecutive low readings
      SerialCom->println("!!! BATTERY CRITICALLY LOW - STOPPING !!!");
      return false; // Consistently low, report error
    } else {
      return true; // Temporarily low, allow operation to continue briefly
    }
  }
}
#else
// Dummy function if battery check disabled
boolean is_battery_voltage_OK() { return true; }
#endif


// Flashes the built-in LED quickly (double flash pattern)
void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis > fast_flash_millis) {
    indexer++;
    if (indexer > 4) { // Off period
      fast_flash_millis = currentMillis + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else { // On/Off flashes
      fast_flash_millis = currentMillis + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    }
  }
}

// Flashes the built-in LED slowly
void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - slow_flash_millis > 1000) { // Toggle every 1 second
    slow_flash_millis = currentMillis;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
  }
}

// Constrains motor power value (simple version)
float constrainPwr(float pwr) {
  float upper = MAX_SPEED; // Use defined MAX_SPEED
  float lower = 50;      // Minimum speed to overcome static friction (adjust)

  if (pwr > upper) return upper;
  if (pwr < lower) return lower; // Return lower bound if input is positive but too small
  // Note: This simple version doesn't handle negative inputs well for lower bound.
  // The overloaded version is generally better.
  return pwr;
}

// Constrains motor power value between lower and upper bounds
float constrainPwr(float pwr, float lower, float upper) {
  if (pwr > upper) return upper;
  if (pwr < lower) return lower;
  return pwr;
}


// Reads serial commands for manual control (Optional)
void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Received Command: "); SerialCom->println(val);

    // Default speed for manual commands
    int manual_speed = 150;

    switch (val) {
      case 'w': forward(manual_speed); SerialCom->println("Manual: Forward"); break;
      case 's': reverse(manual_speed); SerialCom->println("Manual: Reverse"); break;
      case 'a': ccw(manual_speed); SerialCom->println("Manual: CCW"); break;
      case 'd': cw(manual_speed); SerialCom->println("Manual: CW"); break;
      case 'q': strafeLeft(manual_speed); SerialCom->println("Manual: Strafe Left"); break;
      case 'e': strafeRight(manual_speed); SerialCom->println("Manual: Strafe Right"); break;
      case ' ': // Use spacebar to stop
      default:  stop(); SerialCom->println("Manual: Stop"); break;
    }
  }
}

// Dummy function (speed change via serial not implemented here)
void speed_change_smooth() {
  // Placeholder if needed later
}
