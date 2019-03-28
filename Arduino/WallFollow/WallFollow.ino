#include <Wire.h>
#include "WallFollow.h"
#include "IRSensor.h"

// Serial Transmission rate of bits per second
#define BPS 9600
// TCA Multiplexer's Address
#define TCAADDR 0x70

// Sensors configuration (Which sensors exist at which input of the mux)
// Example: 0x13 = 0001 0011. Sensors at SD4/SC4, SD0/SC0 and SD1/SC1
#define VL53SETUP 0x03

// Sensor class / instances
IRSensor** sensors;
WallFollow* alg;
// Maximum amount of sensors allowed.
const int SIZE = 8;
// Error/Info Message
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
  alg = new WallFollow(sensors, true);
  alg->Initialize(0, 1, 100);
  while(1) {
    alg->Act();
    delay(100);
  }
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
  sensors = new IRSensor*[SIZE];

  for (i = 0; i < SIZE; i++) {
      _bit = value % 2;
      value = value / 2;
      sensors[i] = (_bit == 1) ? new IRSensor(false) : NULL;
      if (sensors[i] != NULL && !sensors[i]->success) {
        //If the sensor failed to begin. Print the error message. Then delete the instance.
        sprintf(buf, "Failed to boot VL53L0X #%d.\n", i);
        Serial.println(buf);
        delete sensors[i];
      }
  }
  return;
}
