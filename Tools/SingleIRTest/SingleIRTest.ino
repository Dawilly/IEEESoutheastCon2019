#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include "Adafruit_VL53L0X.h"

#define TCAADDR 0x70

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

/// TCASELECT(uint8_t pin)
///
/// Author: David Weil
/// Description: Sets the I2C Multiplexer (TCA9548A) to change the selected channel.
/// Parameter:
/// @uint8_t pin - The channel to change the Multiplexer to. Valid pins are [0 to 7].
///
/// To-Do: Refactor to a SensorManager class or Multiplexer class.
void TCASELECT(uint8_t pin) {
  if (pin > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << pin);
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  
  Serial.begin(9600);
  Serial.println("\nSingle IR Test ready!");

  /// Set the MUX to read from a particular channel. In this case, channel 0.
  /// See function declaration below.
  // TCASELECT(0);
  
  /// Calling begin on the Adafruit_VL53L0X object will calibrate it and
  /// initialize it for reading.
  if (!lox.begin()) {
    Serial.println(F("Failed to boot LOX VL53L0X"));
    while(1);
  }
}

void loop() {

    /// The measurement object used to obtain data from the IR Sensor.
    VL53L0X_RangingMeasurementData_t measure;

    Serial.print("LOX Reading a measurement... ");

    /// The method used to perform an actual reading. Pass in the Measurement object by reference.
    lox.rangingTest(&measure, false); 

    /// If the RangeStatus is equal to 4, it means that the sensor could not read anything
    /// and that we are out of range of any objects.
    if (measure.RangeStatus != 4) {  
      Serial.print("Distance (mm): "); 
      Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
}
