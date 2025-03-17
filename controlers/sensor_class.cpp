#include <Arduino.h>

class IRSensor {
private:
    int irSensorPin1;         // First IR sensor pin
    int irSensorPin2;         // Second IR sensor pin
    bool serialEnabled;        // Track if serial is enabled
    
    // Calibration constants
    float datasheet_a = 17948.0;
    float datasheet_b = -1.22;
    float calibrated_a = 46161.0;
    float calibrated_b = -1.302;

public:
    // Constructor - initialize with sensor pins
    IRSensor(int pin1 = A0, int pin2 = A1) {
        irSensorPin1 = pin1;
        irSensorPin2 = pin2;
        serialEnabled = false;
    }
    
    // Destructor - cleanup
    ~IRSensor() {
        if (serialEnabled) {
            Serial.end();
        }
    }
    
    // Initialize sensors and serial communication
    void begin(int baudRate = 9600) {
        pinMode(irSensorPin1, INPUT);
        pinMode(irSensorPin2, INPUT);
        Serial.begin(baudRate);
        serialEnabled = true;
    }
    
    // Stop serial communication
    void stopSerial() {
        if (serialEnabled) {
            Serial.end();
            serialEnabled = false;
        }
    }
    
    // Read raw analog value from sensor 1
    int readRawValue1() {
        return analogRead(irSensorPin1);
    }
    
    // Read raw analog value from sensor 2
    int readRawValue2() {
        return analogRead(irSensorPin2);
    }
    
    // Get calculated distance from sensor 1 using datasheet formula
    float getDistanceDatasheet1() {
        int signalADC = readRawValue1();
        return datasheet_a * pow(signalADC, datasheet_b);
    }
    
    // Get calculated distance from sensor 2 using datasheet formula
    float getDistanceDatasheet2() {
        int signalADC = readRawValue2();
        return datasheet_a * pow(signalADC, datasheet_b);
    }
    
    // Get calculated distance from sensor 1 using calibrated formula
    float getDistanceCalibrated1() {
        int signalADC = readRawValue1();
        return calibrated_a * pow(signalADC, calibrated_b);
    }
    
    // Get calculated distance from sensor 2 using calibrated formula
    float getDistanceCalibrated2() {
        int signalADC = readRawValue2();
        return calibrated_a * pow(signalADC, calibrated_b);
    }
    
    // Set calibration constants
    void setCalibration(float ds_a, float ds_b, float cal_a, float cal_b) {
        datasheet_a = ds_a;
        datasheet_b = ds_b;
        calibrated_a = cal_a;
        calibrated_b = cal_b;
    }
    
    // Print sensor values through serial
    void printSensorValues() {
        if (!serialEnabled) return;
        
        int val1 = readRawValue1();
        int val2 = readRawValue2();
        
        Serial.print("Sensor 1 Analog value: ");
        Serial.println(val1);
        Serial.print("Sensor 2 Analog value: ");
        Serial.println(val2);
        
        // Uncomment to print calculated distances
        /*
        Serial.print("Sensor 1 Distance (datasheet): ");
        Serial.print(getDistanceDatasheet1());
        Serial.println(" cm");
        
        Serial.print("Sensor 1 Distance (calibrated): ");
        Serial.print(getDistanceCalibrated1());
        Serial.println(" cm");
        
        Serial.print("Sensor 2 Distance (datasheet): ");
        Serial.print(getDistanceDatasheet2());
        Serial.println(" cm");
        
        Serial.print("Sensor 2 Distance (calibrated): ");
        Serial.print(getDistanceCalibrated2());
        Serial.println(" cm");
        */
    }
    
    // Check for serial commands
    void checkSerialCommands() {
        if (serialEnabled && Serial.available()) {
            byte serialRead = Serial.read();
            if (serialRead == 49) { // ASCII '1'
                stopSerial();
            }
        }
    }
};

// Example usage
IRSensor sensors(A0, A1);  // Initialize with pins A0 and A1

void setup() {
    sensors.begin(9600);  // Start sensors with 9600 baud rate
}

void loop() {
    sensors.checkSerialCommands();  // Check for serial commands
    sensors.printSensorValues();    // Print current sensor values
    delay(1000);                   // Wait for 1 second
}