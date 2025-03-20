#include <Servo.h> //Need for Servo pulse output

// #define NO_READ_GYRO  //Uncomment of GYRO is not attached.
// #define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
// #define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

// State machine states
enum STATE
{
    INITIALISING,
    RUNNING,
    STOPPED
};

// Refer to Shield Pinouts.jpg for pin locations

// Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

class IRSensor
{
private:
    int sensorPin;
    const char *sensorName;

public:
    // Constructor - initialize with sensor pin
    IRSensor(int pin, const char *name)
    {
        sensorPin = pin;
        sensorName = name;
    }

    // Initialize sensor
    void begin()
    {
        pinMode(sensorPin, INPUT);
    }

    // Read raw analog value from sensor
    int readRawValue()
    {
        return analogRead(sensorPin);
    }

    // Get sensor name
    const char *getName()
    {
        return sensorName;
    }
};

class Gyro
{
private:
    int sensorPin;                 // Pin connected to the gyro
    float currentAngle = 0;        // Current angle in degrees
 
public:
   unsigned long lastTime;        // Last time the gyro was read
    float gyroZeroVoltage;         // Zero-drift voltage of the gyro
    float gyroSensitivity = 0.007; // Sensitivity in V/dps (from datasheet)
    float rotationThreshold = 1.5; // Minimum angular velocity to consider (dps)
    float gyroSupplyVoltage = 5.0; // Supply voltage for the gyro

    // Constructor - initialize with sensor pin
    Gyro(int pin)
    {
        sensorPin = pin; // Initialize the sensor pin
        currentAngle = 0;
        lastTime = 0;
    }

    // Reset the gyro angle to 0
    void reset()
    {
        currentAngle = 0;
        lastTime = millis();
        Serial.println("Gyro reset: Angle set to 0");
    }

    // Initialize the gyro
    void begin()
    {
        pinMode(sensorPin, INPUT); // Set the sensor pin as input
        reset();                   // Reset the gyro angle to 0

        // Calibrate the gyro to find the zero-drift voltage
        int i;
        float sum = 0;
        float sensorValue;
        Serial.println("Please keep the sensor still for calibration...");
        for (i = 0; i < 100; i++)
        { // Read 100 values to calculate the zero-drift
            sensorValue = analogRead(sensorPin);
            sum += sensorValue;
            delay(5);
        }
        gyroZeroVoltage = (sum / 100) * (gyroSupplyVoltage / 1023.0); // Convert to voltage
        Serial.print("Gyro Zero Voltage: ");
        Serial.println(gyroZeroVoltage, 4);
    }

    // Read the angular velocity from the gyro
    float readAngularVelocity()
    {
        // Read the raw analog value and convert to voltage
        float sensorValue = analogRead(sensorPin);
        float voltage = (sensorValue / 1023.0) * gyroSupplyVoltage;

        // Calculate angular velocity (degrees per second)
        float angularVelocity = (voltage - gyroZeroVoltage) / gyroSensitivity;

        // Apply a threshold to ignore small noise
        if (abs(angularVelocity) < rotationThreshold)
        {
            angularVelocity = 0;
        }

        // Debug: Print the raw value, voltage, and angular velocity
        Serial.print("Raw Value: ");
        Serial.print(sensorValue);
        Serial.print(", Voltage: ");
        Serial.print(voltage, 4);
        Serial.print(", Angular Velocity: ");
        Serial.println(angularVelocity, 4);

        return angularVelocity;
    }

    // Update the current angle based on the angular velocity
    void updateAngle()
    {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        // Calculate the angular velocity
        float angularVelocity = readAngularVelocity();

        // Update the current angle
        currentAngle += angularVelocity * deltaTime;

        // Debug: Print the delta time and current angle
        Serial.print("Delta Time: ");
        Serial.print(deltaTime, 4);
        Serial.print(" s, Current Angle: ");
        Serial.println(currentAngle, 4);
    }

    // Get the current angle
    float getAngle()
    {
        return currentAngle;
    }
};

IRSensor sensor1(A4, "Front"); // Front sensor on A4
IRSensor sensor2(A5, "Back");  // Back sensor on A5

Gyro gyro(A3); // Gyro on A3

// Speed control
int speed_val = 250;          // Fixed speed
char current_direction = ' '; // Current movement direction

const float ROTATION_DELAY_PER_DEGREE = 11.875;
const int SENSOR_OFFSET = 61;
const float SENSOR_SCALE = 4261.4;
const int SENSOR_MIN_THRESHOLD = 110;
const int SENSOR_MAX_THRESHOLD = 500;
const int TIMEOUT_DURATION = 10000;

// Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor; // create servo object to control Vex Motor Controller 29
Servo right_font_motor; // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int speed_change;

// Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
    turret_motor.attach(11);
    pinMode(LED_BUILTIN, OUTPUT);

    // The Trigger pin will tell the sensor to range find
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);

    // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
    SerialCom = &Serial;
    SerialCom->begin(115200);
    SerialCom->println("MECHENG706_Base_Code_25/01/2018");
    delay(1000);
    SerialCom->println("Setup....");

    delay(1000); // settling time but no really needed

rotate(90);

}

void loop(void) // main loop
{
    static STATE machine_state = INITIALISING;
    // Finite-state machine Code
    switch (machine_state)
    {
    case INITIALISING:
        machine_state = initialising();
        break;
    case RUNNING: // Lipo Battery Volage OK
        machine_state = running();
        break;
    case STOPPED: // Stop of Lipo Battery voltage is too low, to protect Battery
        machine_state = stopped();
        break;
    };
}

STATE initialising()
{
    // initialising
    SerialCom->println("INITIALISING....");
    delay(1000); // One second delay to see the serial string "INITIALISING...."
    SerialCom->println("Enabling Motors...");
    enable_motors();
    SerialCom->println("RUNNING STATE...");
    return RUNNING;
}

STATE running()
{

    static unsigned long previous_millis;

    read_serial_command();
    fast_flash_double_LED_builtin();

    if (millis() - previous_millis > 500)
    { // Arduino style 500ms timed execution statement
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
        if (!is_battery_voltage_OK())
            return STOPPED;
#endif

        turret_motor.write(pos);

        if (pos == 0)
        {
            pos = 45;
        }
        else
        {
            pos = 0;
        }
    }

    return RUNNING;
}

// Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped()
{
    static byte counter_lipo_voltage_ok;
    static unsigned long previous_millis;
    int Lipo_level_cal;
    disable_motors();
    slow_flash_LED_builtin();

    if (millis() - previous_millis > 500)
    { // print massage every 500ms
        previous_millis = millis();
        SerialCom->println("STOPPED---------");

#ifndef NO_BATTERY_V_OK
        // 500ms timed if statement to check lipo and output speed settings
        if (is_battery_voltage_OK())
        {
            SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
            SerialCom->println(counter_lipo_voltage_ok);
            counter_lipo_voltage_ok++;
            if (counter_lipo_voltage_ok > 10)
            { // Making sure lipo voltage is stable
                counter_lipo_voltage_ok = 0;
                enable_motors();
                SerialCom->println("Lipo OK returning to RUN STATE");
                return RUNNING;
            }
        }
        else
        {
            counter_lipo_voltage_ok = 0;
        }
#endif
    }
    return STOPPED;
}

void fast_flash_double_LED_builtin()
{
    static byte indexer = 0;
    static unsigned long fast_flash_millis;
    if (millis() > fast_flash_millis)
    {
        indexer++;
        if (indexer > 4)
        {
            fast_flash_millis = millis() + 700;
            digitalWrite(LED_BUILTIN, LOW);
            indexer = 0;
        }
        else
        {
            fast_flash_millis = millis() + 100;
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
}

void slow_flash_LED_builtin()
{
    static unsigned long slow_flash_millis;
    if (millis() - slow_flash_millis > 2000)
    {
        slow_flash_millis = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

void speed_change_smooth()
{
    speed_val += speed_change;
    if (speed_val > 1000)
        speed_val = 1000;
    speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
    static byte Low_voltage_counter;
    static unsigned long previous_millis;

    int Lipo_level_cal;
    int raw_lipo;
    // the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
    // to about 4.20-4.25V (fully charged) = 860(4.2V Max)
    // Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
    raw_lipo = analogRead(A0);
    Lipo_level_cal = (raw_lipo - 717);
    Lipo_level_cal = Lipo_level_cal * 100;
    Lipo_level_cal = Lipo_level_cal / 143;

    if (Lipo_level_cal > 0 && Lipo_level_cal < 160)
    {
        previous_millis = millis();
        SerialCom->print("Lipo level:");
        SerialCom->print(Lipo_level_cal);
        SerialCom->print("%");
        // SerialCom->print(" : Raw Lipo:");
        // SerialCom->println(raw_lipo);
        SerialCom->println("");
        Low_voltage_counter = 0;
        return true;
    }
    else
    {
        if (Lipo_level_cal < 0)
            SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
        else if (Lipo_level_cal > 160)
            SerialCom->println("!Lipo is Overchanged!!!");
        else
        {
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

#ifndef NO_HC - SR04
void HC_SR04_range()
{
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
    while (digitalRead(ECHO_PIN) == 0)
    {
        t2 = micros();
        pulse_width = t2 - t1;
        if (pulse_width > (MAX_DIST + 1000))
        {
            SerialCom->println("HC-SR04: NOT found");
            return;
        }
    }

    // Measure how long the echo pin was held high (pulse width)
    // Note: the micros() counter will overflow after ~70 min

    t1 = micros();
    while (digitalRead(ECHO_PIN) == 1)
    {
        t2 = micros();
        pulse_width = t2 - t1;
        if (pulse_width > (MAX_DIST + 1000))
        {
            SerialCom->println("HC-SR04: Out of range");
            return;
        }
    }

    t2 = micros();
    pulse_width = t2 - t1;

    // Calculate distance in centimeters and inches. The constants
    // are found in the datasheet, and calculated from the assumed speed
    // of sound in air at sea level (~340 m/s).
    cm = pulse_width / 58.0;
    inches = pulse_width / 148.0;

    // Print out results
    if (pulse_width > MAX_DIST)
    {
        SerialCom->println("HC-SR04: Out of range");
    }
    else
    {
        SerialCom->print("HC-SR04:");
        SerialCom->print(cm);
        SerialCom->println("cm");
    }
}
#endif

void Analog_Range_A4()
{
    SerialCom->print("Analog Range A4:");
    SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
    SerialCom->print("GYRO A3:");
    SerialCom->println(analogRead(A3));
}

#endif

// Serial command pasing
void read_serial_command()
{
    if (SerialCom->available())
    {
        char val = SerialCom->read();
        SerialCom->print("Speed:");
        SerialCom->print(speed_val);
        SerialCom->print(" ms ");

        // Perform an action depending on the command
        switch (val)
        {
        case 'w': // Move Forward
        case 'W':
            forward();
            SerialCom->println("Forward");
            break;
        case 's': // Move Backwards
        case 'S':
            reverse();
            SerialCom->println("Backwards");
            break;
        case 'q': // Turn Left
        case 'Q':
            strafe_left();
            SerialCom->println("Strafe Left");
            break;
        case 'e': // Turn Right
        case 'E':
            strafe_right();
            SerialCom->println("Strafe Right");
            break;
        case 'a': // Turn Right
        case 'A':
            ccw();
            SerialCom->println("ccw");
            break;
        case 'd': // Turn Right
        case 'D':
            cw();
            SerialCom->println("cw");
            break;
        case '-': // Turn Right
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
// The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
    left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
    left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
    right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
    right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

    pinMode(left_front, INPUT);
    pinMode(left_rear, INPUT);
    pinMode(right_rear, INPUT);
    pinMode(right_front, INPUT);
}

void enable_motors()
{
    left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
    left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
    right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
    right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() // Stop
{
    left_font_motor.writeMicroseconds(1500);
    left_rear_motor.writeMicroseconds(1500);
    right_rear_motor.writeMicroseconds(1500);
    right_font_motor.writeMicroseconds(1500);
}

void forward()
{
    left_font_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse()
{
    left_font_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw()
{
    left_font_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw()
{
    left_font_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left()
{
    left_font_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right()
{
    left_font_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_font_motor.writeMicroseconds(1500 + speed_val);
}

void rotate(int degrees)
{
    // Reset the gyro angle to 0
    gyro.reset();

    // Determine the direction of rotation
    if (degrees > 0)
    {
        ccw(); // Counter-clockwise rotation
    }
    else
    {
        cw(); // Clockwise rotation
    }

    // Initialize variables for tracking the angle turned
    float currentAngle = 0;
    unsigned long previousTime = millis();

    // Rotate until the desired angle is reached
    while (abs(currentAngle) < abs(degrees))
    {
        // Use the Gyro class's method to read angular velocity
        float angularVelocity = gyro.readAngularVelocity();

        // Ignore small angular velocities below the threshold
        if (angularVelocity >= gyro.rotationThreshold || angularVelocity <= -gyro.rotationThreshold)
        {
            // Calculate the time elapsed since the last reading
            unsigned long currentTime = millis();
            float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
            previousTime = currentTime;

            // Calculate the change in angle
            float angleChange = angularVelocity * deltaTime;
            currentAngle += angleChange;

            // Debug: Print the current angle and angular velocity
            Serial.print("Angular Velocity: ");
            Serial.print(angularVelocity);
            Serial.print(" dps, Current Angle: ");
            Serial.println(currentAngle);
        }
    }

    // Stop the robot once the desired angle is reached
    stop();
}

// Convert raw sensor value to distance - don't use this for decisions
float calculateDistance(int rawValue)
{
    return (rawValue - SENSOR_OFFSET) / SENSOR_SCALE;
}

void forwardUntil(int targetDistance, int speed)
{
    // Set the speed for forward movement until robot is a set distance away
    speed_val = speed;

    // Tmeout feature to turn off after a certain time automatically
    unsigned long startTime = millis();

    while (true)
    {
        if (millis() - startTime > TIMEOUT_DURATION)
        {
            Serial.println("Timeout reached. Stopping.");
            break;
        }

        // Read the current distance from the front sensor
        int currentRawValue = sensor1.readRawValue();

        // Convert the raw value to distance
        float currentDistance = calculateDistance(currentRawValue);
        Serial.print("Current Distance: ");
        Serial.println(currentDistance);

        // Check if the target distance has been reached
        if ((currentDistance <= targetDistance) &&
            (currentRawValue > SENSOR_MIN_THRESHOLD && currentRawValue < SENSOR_MAX_THRESHOLD))
        {
            Serial.print("Target Distance Reached: ");
            Serial.println(currentDistance);
            break;
        }

        // Move forward
        forward();
    }

    // Stop the robot once the target distance is reached
    stop();
}
