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
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox7 = Adafruit_VL53L0X();

int online = 0;
int online1 = 0;
int online2 = 0;
int online3 = 0;
int online4 = 0;
int online5 = 0;
int online6 = 0;
int online7 = 0;

#define TCAADDR 0x70

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
    
    Serial.begin(9600);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      Serial.print("t = ");
      Serial.print(t);
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }

  tcaselect(0);
  if (!lox.begin()) {
    Serial.println(F("Failed to boot LOX VL53L0X"));
    //while(1);
  } else {
    online = 1;
  }

  tcaselect(1);
  if (!lox1.begin()) {
    Serial.println(F("Failed to boot LOX1 VL53L0X"));
    //while(1);
  } else {
    online1 = 1;
  }

  tcaselect(2);
  if (!lox2.begin()) {
    Serial.println(F("Failed to boot LOX2 VL53L0X"));
    //while(1);
  } else {
    online2 = 1;
  }

  tcaselect(3);
  if (!lox3.begin()) {
    Serial.println(F("Failed to boot LOX3 VL53L0X"));
    //while(1);
  } else {
    online3 = 1;
  }

  tcaselect(4);
  if (!lox4.begin()) {
    Serial.println(F("Failed to boot LOX4 VL53L0X"));
    //while(1);
  } else {
    online4 = 1;
  }

  tcaselect(5);
  if (!lox5.begin()) {
    Serial.println(F("Failed to boot LOX5 VL53L0X"));
    //while(1);
  } else {
    online5 = 1;
  }

  tcaselect(6);
  if (!lox6.begin()) {
    Serial.println(F("Failed to boot LOX6 VL53L0X"));
    //while(1);
  }else {
    online6 = 1;
  }

  tcaselect(7);
  if (!lox7.begin()) {
    Serial.println(F("Failed to boot LOX7 VL53L0X"));
    //while(1);
  }else {
    online7 = 1;
  }
  
   Serial.println("\ndone");
}

void loop() {
  tcaselect(0);
  VL53L0X_RangingMeasurementData_t measure;

  if (online) {
    Serial.print("LOX Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }

  delay(100);

  tcaselect(1);
  if (online1) {
    Serial.print("LOX 1 Reading a measurement... ");
    lox1.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
    
  delay(100);

  tcaselect(2);
  if (online2) {
    Serial.print("lOX2 Reading a measurement... ");
    lox2.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
    
  delay(100);

  tcaselect(3);
  if (online3) {
    Serial.print("LOX3 Reading a measurement... ");
    lox3.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
  delay(100);

  tcaselect(4);
  if (online4) {
    Serial.print("LOX4 Reading a measurement... ");
    lox4.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
    
  delay(100);

  tcaselect(5);
  if (online5) {
    Serial.print("LOX5 Reading a measurement... ");
    lox5.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
    
  delay(100);

  tcaselect(6);
  if (online6) {
    Serial.print("LOX6 Reading a measurement... ");
    lox6.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
    
  delay(100);

  tcaselect(7);
  if (online7) {
    Serial.print("LOX7 Reading a measurement... ");
    lox7.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
  }
    
  delay(100);
}
