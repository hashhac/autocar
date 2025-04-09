volatile int value_x = 0;
volatile int value_y = 0;

#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11


#define LOOP_DELAY 10 // miliseconds
#define SAMPLE_DELAY 10 // miliseconds

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

volatile int32_t Counter_for_blue_tooth = 1;
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  
  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  
  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  
  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3)
{
  if (OUTPUTMONITOR)
  {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER)
  {
    serialOutputPlotter(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR)
  {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);;
  }
}
void blutooth_call(void) {
  serialOutput(Counter_for_blue_tooth, value_x, value_y);
  Counter_for_blue_tooth = Counter_for_blue_tooth + 1;
}
void update_cords(int x, int y) {
    value_x = x;
    value_Y = y;
  blutooth_call();
}