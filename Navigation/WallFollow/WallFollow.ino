#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// Serial Transmission rate of bits per second
#define BPS 9600
// TCA Multiplexer's Address
#define TCAADDR 0x70

// Sensors configuration (Which sensors exist at which input of the mux)
// Example: 0x13 = 0001 0011. Sensors at SD4/SC4, SD0/SC0 and SD1/SC1
#define VL53SETUP 0x03

// Sensor class / instances
Adafruit_VL53L0X** sensors;
// Measurement class / instances
VL53L0X_RangingMeasurementData_t** measurements;
// Maximum amount of sensors allowed.
const int SIZE = 8;
// Print buffer cstring. Since Serial is weird. Dawg.
char buf[100];

///setup()
///
///Description: Initial function when Arduino boots. The very first thing that is done.
void setup() {
  // Wait for Serial to exist
  while(!Serial);
  delay(500);
  
  Wire.begin();
  Serial.begin(BPS);
  Serial.println("\nWall Follow algorithm.");
  Serial.print("Initalizing...");
  
  setupSensors(VL53SETUP);

  Serial.println("Ready!");
}

///loop()
///
///Description: The main loop. Starts after setup() is done.
void loop() {
  // Potential Kill Switch
  bool runRobot = false;
  bool needAdjusting = false;

  //Time varying variables.
  unsigned long setTime = 0;
  unsigned long elapsedTime = 0;
  //Time variables used to calculate the time it took for a single loop.
  //Added into elapsedTime.
  unsigned long beginTime = 0;
  unsigned long endTime = 0;

  //Get the amount of time the robot should run (in milliseconds)
  if (Serial.available() > 0) {
    Serial.println("Enter how long the robot should run (in milliseconds)");
    setTime = (unsigned long) Serial.parseInt();
    runRobot = true;
  }
  
  while (runRobot && (setTime > elapsedTime)) {
    beginTime = micros();

    //Check the two sensors facing the wall.
    needAdjusting = checkSensors(0, 1, 50);
    if (needAdjusting) {
      // Adjustment is needed, as the sensors determined threshold has been surpassed.
      
      // KATIE: Call for function/instance that can adjust robot
      // KATIE: The measurements will be in measurements[0] and measurements[1]. Assuming
      // the first two parameters of checkSensors(int, int, uint16_t) are 0 and 1.
    }
    // KATIE: Move robot forward.
    
    endTime = micros();

    //Add the time it took for this loop iteration.
    elapsedTime += (endTime - beginTime);
  }

  // Should be set to false during the next iteration of loop, but just in case
  // of optimization...
  runRobot = false;
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

  return ((measurements[s1]->RangeMilliMeter + measurements[s2]->RangeMilliMeter) <= threshold);
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
