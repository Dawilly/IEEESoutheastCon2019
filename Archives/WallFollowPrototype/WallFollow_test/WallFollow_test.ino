#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "PID_v1.h"
#include "Encoder.h"
#include "Motors.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

// Status pin for Raspberry Pi
#define STATUS_PIN 30
// Serial Transmission rate of bits per second
#define BPS 9600
// TCA Multiplexer's Address
#define TCAADDR 0x70
// Sensors configuration (Which sensors exist at which input of the mux)
// Example: 0x13 = 0001 0011. Sensors at SD4/SC4, SD0/SC0 and SD1/SC1
#define VL53SETUP 0x03

//Set the delay between samples of the IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Sensor class / instances
Adafruit_VL53L0X** sensors;
// Measurement class / instances
VL53L0X_RangingMeasurementData_t** measurements;
// Maximum amount of sensors allowed.
const int SIZE = 8;
// Sensor placement
const int SENSOR_0 = 0;
const int SENSOR_1 = 1;
// Print buffer cstring. Since Serial is weird. Dawg.
char buf[100];

// Motor(MotorPWM,MotorIn1,MotorIn2,EncoderA,EncoderB)
Motor M[2] = 
{ 
  {9,6,7,2,4}, // right motor
  {10,33,35,3,5} // left motor
};

int Speed = 0;
unsigned long lastMilli = 0;
double xVal;
double xValInitial;

///setup()
///
///Description: Initial function when Arduino boots. The very first thing that is done.
void setup() {
  // Wait for Serial to exist
  while(!Serial);
  delay(500);
  
  Wire.begin();
  Serial.begin(BPS, SERIAL_8N1);
  Serial.println("\nWall Follow algorithm.");
  Serial.print("Initalizing...");

  //Katie's Set up
  //////
  pinMode(STATUS_PIN, OUTPUT);
  Speed = 200;

  // Setup the IMU
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(300);
  //Get initial heading from IMU
  bno.setExtCrystalUse(true);
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("Initial X: ");
  xValInitial = event.orientation.x;
  Serial.print(xValInitial, 4);
  Serial.print("\n");
  delay(500);
  sensors_event_t event2;
  bno.getEvent(&event2);
  Serial.print("Initial X: ");
  xValInitial = event2.orientation.x;
  Serial.print(xValInitial, 4);
  Serial.print("\n");

  // Status is HIGH when ready for commands, and LOW when processing commands
  digitalWrite(STATUS_PIN, HIGH);
  //////

  //David's Set up
  setupSensors(VL53SETUP);

  Serial.println("Ready!");
}

///loop()
///
///Description: The main loop. Starts after setup() is done.
void loop() {
  // Potential Kill Switch
  bool runRobot = false;

  //Time varying variables.
  unsigned long setDistance = 0;

  //Get the amount of time the robot should run (in milliseconds)
  if (Serial.available() > 0) {
    Serial.println("Enter how long the robot should run (in milliseconds)");
    setDistance = Serial.parseInt();
    driveDistance(setDistance);
  }
}

/// checkSensors(int s1, int s2, uint16_t threshold)
///
/// Description: Takes one measurement from two different sensors, each.
///              Calculates the difference and compares with threshold.
/// Parameters:
/// @int s1 - The first sensor to take a measurement from.
/// @int s2 - The second sensor to take a measurement from.
/// @uint16_t threshold - The maximum difference allowed between the two
///                       distance readings.
/// Return:
/// True - If the measurement between s1 and s2 is less than or equal to 
///        the threshold.
/// False - Otherwise.
bool checkSensors(int s1, int s2, uint16_t threshold) {
  tcaselect(s1);
  delay(100);
  sensors[s1]->rangingTest(measurements[s1], false);
  printSensorData(s1);
  sensors[s2]->rangingTest(measurements[s2], false);
  printSensorData(s2);
  
  return (abs(measurements[s1]->RangeMilliMeter - measurements[s2]->RangeMilliMeter) >= threshold);
}

uint16_t sensorDiff(int s1, int s2) {
  return measurements[s1]->RangeMilliMeter - measurements[s2]->RangeMilliMeter;
}

/// printSensorData(int pin)
///
/// Description: Prints out the data the sensors measured.
/// Parameters:
/// @int pin - The pin number containing the sensor we want to read.
void printSensorData(int pin) {
  if (measurements[pin] != NULL && measurements[pin]->RangeStatus != 4) {
    sprintf(buf, "Distance (mm): %d\n", measurements[pin]->RangeMilliMeter);
    Serial.print(buf);
  } else if (measurements[pin] == NULL) {
    sprintf(buf, "Pin %d does not contain a sensor/failed to boot\n", pin);
    Serial.print(buf);
  } else {
    sprintf(buf, "Sensor %d is out of range\n", pin);
    Serial.print(buf);
  }
  return;
}

/// setupSensors(uint8_t value)
///
/// Description: Allocates and initalizes the VL53L0X sensors and Measurement objects.
/// Parameter:
/// @uint8_t value - Informs which sensors exist at which pins of the Multiplex.
///                  See VL53SETUP defintion for detail and example.  
void setupSensors(uint8_t value) {
  int i;
  int _bit;
  sensors = new Adafruit_VL53L0X*[SIZE];
  measurements = new VL53L0X_RangingMeasurementData_t*[SIZE];

  for (i = 0; i < SIZE; i++) {
      _bit = value % 2;
      value = value / 2;
      sensors[i] = (_bit == 1) ? new Adafruit_VL53L0X() : NULL;
      measurements[i] = (_bit == 1) ? new VL53L0X_RangingMeasurementData_t() : NULL;
      if (sensors[i] != NULL) {
        if (!sensors[i]->begin()) {
          //If the sensor failed to begin. Print the error message. Then delete the instance.
          sprintf(buf, "Failed to boot VL53L0X #%d.\n", i);
          Serial.println(buf);
          delete sensors[i];
          delete measurements[i];
          sensors[i] = NULL;
          measurements[i] = NULL;
        }
      }
  }
  return;
}

/// tcaselect(uint8_t pin)
///
/// Description: Sets the active input pin on the multiplexer. 
/// Parameter:
/// @uint8_t pin - The input pin(s) of the multiplexer to set. Valid values range
///                from 0 to 7. Values above 7 are ignored.
void tcaselect(uint8_t pin) {
  if (pin > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << pin);
  Wire.endTransmission();
}

///
///Katie's Code
///

void driveDistance(int distance) {
  M[0].resetPosition();
  M[1].resetPosition();
  Speed = 200;
  
  double tickGoal = inchesToTicks(distance);
  double posAvg = 0.0;
  bool needAdjusting = false;
  
  while(1) {
    //Katie's Original
    //double posRight = abs(M[0].getPosition());
    //double posLeft = abs(M[1].getPosition());
    //double posAvg = (posRight + posLeft) / 2.0;

    //David's Modification
    posAvg = calculatePosAvg();
    needAdjusting = checkSensors(SENSOR_1, SENSOR_0, 50);
    if (needAdjusting) {
      //Formula bases
      Serial.println("Needs adjusting!");
    }
    
    if(posAvg < tickGoal) {
      M[0].run(FORWARD); //right motor
      M[1].run(FORWARD); //left motor
      M[0].Setpoint = Speed;
      M[1].Setpoint = Speed;
    } else {
      for(int j=0;j<2;j++) {
        M[j].run(STOP);
        M[j].Setpoint = 0;
      }
      break;
    }
    
    if((millis()-lastMilli) >= LOOPTIME) {
      lastMilli = millis();
      M[0].updatePID();
      M[1].updatePID();         
    }    
  }
}

double inchesToTicks(int inches) {
  double ticksPerInch = 2700.0 / CIRCUMFERENCE;
  return inches * ticksPerInch;
}

double calculatePosAvg() {
  double right = abs(M[0].getPosition());
  double left = abs(M[1].getPosition());
  return ((right + left) / 2.0);
}
