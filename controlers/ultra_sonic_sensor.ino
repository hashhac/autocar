#include <Servo.h>

class UltrasonicSensorMovement {
public:
  UltrasonicSensorMovement(Servo& servo, HardwareSerial& serial, int trigPin, int echoPin, int minAngle, int maxAngle, int angleIncrement)
      : myservo_(servo), SerialCom_(serial), trigPin_(trigPin), echoPin_(echoPin), sweepMin_(minAngle), sweepMax_(maxAngle), sweepIncrement_(angleIncrement), arraySize_((sweepMax_ - sweepMin_) / sweepIncrement_ + 1) {
    pinMode(trigPin_, OUTPUT);
    digitalWrite(trigPin_, LOW);
  }

  void sweepAndDetectCorner() {
    float distances[arraySize_];
    sweep(distances);
    detectCorner(distances);
  }

  float getDistance() {
    return HC_SR04_range();
  }

  void moveToAngle(int angle) {
    myservo_.write(angle);
  }

private:
  Servo& myservo_;
  HardwareSerial& SerialCom_;
  int trigPin_;
  int echoPin_;
  int sweepMin_;
  int sweepMax_;
  int sweepIncrement_;
  int arraySize_;
  const unsigned int MAX_DIST = 23200;


  void sweep(float* dist) {
    int count = 0;
    myservo_.write(0);
    delay(25);
    for (int pos = sweepMin_; pos <= sweepMax_; pos += sweepIncrement_) {
      myservo_.write(pos);
      SerialCom_.print(pos);
      SerialCom_.print(": ");
      dist[count] = HC_SR04_range();
      count++;
      delay(200);
    }
    myservo_.write(90);
    delay(2000);
  }

  void detectCorner(float* array) {
    float prev_diff = 0;
    float max_diff = prev_diff;
    int index = 0;

    for (int j = 1; j < arraySize_; j++) {
      if ((array[j] != -1) && (array[j - 1] != -1)) {
        float current_diff = abs(array[j] - array[j - 1]);
        if ((current_diff < 0.4 * array[j]) || (current_diff < 0.4 * array[j - 1])) {
          if (current_diff > max_diff) {
            max_diff = current_diff;
            index = j;
          }
        }
        prev_diff = current_diff;
      }
    }

    float found_angle = sweepMin_ + index * sweepIncrement_ - 0.5 * sweepIncrement_;
    myservo_.write(found_angle);
  }

  float HC_SR04_range() {
    digitalWrite(trigPin_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_, LOW);

    unsigned long t1 = micros();
    while (digitalRead(echoPin_) == 0 && micros() - t1 < MAX_DIST + 1000);
    if (digitalRead(echoPin_) == 0) return -1;

    t1 = micros();
    while (digitalRead(echoPin_) == 1 && micros() - t1 < MAX_DIST + 1000);
    if (digitalRead(echoPin_) == 1) return -1;

    unsigned long pulse_width = micros() - t1;
    return pulse_width / 58.0;
  }
};

// Example usage:
Servo myservo;
HardwareSerial *SerialCom;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
int sweep_min = 45;
int sweep_max = 135;
int sweep_increment = 5;

UltrasonicSensorMovement ultrasonicSensor(myservo, Serial, TRIG_PIN, ECHO_PIN, sweep_min, sweep_max, sweep_increment);

void setup() {
  myservo.attach(7);
  SerialCom = &Serial;
  SerialCom->begin(115200);
}

void loop() {
  // Example of using moveToAngle function
  Serial.println("Moving to 90 degrees...");
  ultrasonicSensor.moveToAngle(90);
  delay(1000);

  Serial.println("reading distance...");
  Serial.println(ultrasonicSensor.getDistance());

  //move to 0 degrees
  Serial.println("Moving to 0 degrees...");
  ultrasonicSensor.moveToAngle(0);
  delay(1000);

  Serial.println("reading distance...");
  Serial.println(ultrasonicSensor.getDistance());

  //moving to 180 degrees
  Serial.println("Moving to 180 degrees...");
  ultrasonicSensor.moveToAngle(180);
  delay(1000);

  Serial.println("reading distance...");
  Serial.println(ultrasonicSensor.getDistance());

//   Serial.println("Sweeping and detecting corner...");
//   ultrasonicSensor.sweepAndDetectCorner();
//   delay(5000); // Delay for demonstration
}