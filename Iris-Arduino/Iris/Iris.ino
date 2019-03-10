#include <Adafruit_VL53L0X.h>


/**
 * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
 *
 * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
 *
 */

#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
 
#define TCAADDR 0x70

Adafruit_VL53L0X** lox;
VL53L0X_RangingMeasurementData_t** measure;
 
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
 
 
// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);
 
    Wire.begin();
    lox = new Adafruit_VL53L0X*[8];
    measure = new VL53L0X_RangingMeasurementData_t*[8];
    for (int i = 0; i < 8; i++) {
      lox[i] = NULL;
      measure[i] = NULL;
    }
    Serial.begin(115200);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);
 
      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
           lox[t] = new Adafruit_VL53L0X();
           delay(100);
           if (!lox[t]->begin()) {
             Serial.println(F("Failed to boot VL53L0X"));
             lox[t] = NULL;
             continue;
           }
           measure[t] = new VL53L0X_RangingMeasurementData_t;
        }
      }
    }
    
    delay(1000);
    Serial.println("\ndone");
}
 
void loop() 
{
  
  for (uint8_t i=0; i < 8; i++) {
    if (lox[i] == NULL) continue;
    tcaselect(i);
    lox[i]->rangingTest(measure[i], false);
    Serial.print("Sensor #");
    Serial.print(i);
    Serial.print(": ");
    if (measure[i]->RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); 
      Serial.println((i == 1) ? (measure[i]->RangeMilliMeter) : measure[i]->RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
      
    delay(1000);
  }
}
