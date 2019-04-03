#include <PID_v1.h>
#include <Encoder.h>
#include "Motors.h"
#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include "Adafruit_VL53L0X.h"

Motor M[2] = 
{ 
  {9,6,7,2,4}, // right motor
  {10,33,35,3,5} // left motor
};

Adafruit_VL53L0X bottomLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X topLeft = Adafruit_VL53L0X();

int Speed = 0;
unsigned long lastMilli = 0;

#define TCAADDR 0x70

boolean runOnce = false;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
    while (!Serial);
    delay(1000);

    Wire.begin();
    
    Serial.begin(9600);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
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
  if (!bottomLeft.begin()) {
    Serial.println(F("Failed to boot bottom left VL53L0X"));
    while(1);
  }

  tcaselect(1);
  if (!topLeft.begin()) {
    Serial.println(F("Failed to boot top left VL53L0X"));
    while(1);
  }

}

void loop() {
  delay(5000);
  
  if(!runOnce) {
    //takeMeasurements();
    wallFollowLeft(72);
    //driveDistance(72);
    runOnce = true;
  }
}

void driveDistance(int distance) {
  M[0].resetPosition();
  M[1].resetPosition();
  Speed = 100;
  double tickGoal = inchesToTicks(distance);
  while(1) {
    double posRight = abs(M[0].getPosition());
    double posLeft = abs(M[1].getPosition());
    double posAvg = (posRight + posLeft) / 2.0;
    if(posAvg < tickGoal) {
      M[0].run(FORWARD); //right motor
      M[1].run(FORWARD); //left motor
      M[0].Setpoint = Speed;
      M[1].Setpoint = Speed;
    }
    else {
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


void takeMeasurements() {
  boolean bottomLeftOutOfRange;
  boolean topLeftOutOfRange;
  
  VL53L0X_RangingMeasurementData_t bottomLeftMeasurement;
  VL53L0X_RangingMeasurementData_t topLeftMeasurement;

  uint16_t bottomLeftRange;
  uint16_t topLeftRange;

  while(1) {
    Serial.println("-------------");
      tcaselect(0);
      Serial.println("Reading a measurement (bottom left sensor)... ");
      bottomLeft.rangingTest(&bottomLeftMeasurement, false); // pass in 'true' to get debug data printout!
      if(bottomLeftMeasurement.RangeStatus != 4) {
        bottomLeftOutOfRange = false;
        bottomLeftRange = bottomLeftMeasurement.RangeMilliMeter;
        Serial.println(bottomLeftRange);
      }
      else {
        bottomLeftOutOfRange = true;
      }

      delay(100);
      
      tcaselect(1);
      Serial.println("Reading a measurement (top left sensor)... ");
      topLeft.rangingTest(&topLeftMeasurement, false); // pass in 'true' to get debug data printout!
      if(topLeftMeasurement.RangeStatus != 4) {
        topLeftOutOfRange = false;
        topLeftRange = topLeftMeasurement.RangeMilliMeter;
        Serial.println(topLeftRange);
        Serial.println("-------------");
      }
  }
}

void wallFollowLeft(int distance) {
  boolean bottomLeftOutOfRange;
  boolean topLeftOutOfRange;
  int superThreshold = 30;
  int normalThreshold = 10;
  
  M[0].resetPosition();
  M[1].resetPosition();

  VL53L0X_RangingMeasurementData_t bottomLeftMeasurement;
  VL53L0X_RangingMeasurementData_t topLeftMeasurement;

  uint16_t bottomLeftRange;
  uint16_t topLeftRange;
  
  int rightSpeed = 100;
  int leftSpeed = 100;
  double tickGoal = inchesToTicks(distance);
  
  while(1) {
    double posRight = abs(M[0].getPosition());
    double posLeft = abs(M[1].getPosition());
    double posAvg = (posRight + posLeft) / 2.0;
    if(posAvg < tickGoal) {
      Serial.println("-------------");
      tcaselect(0);
      Serial.println("Reading a measurement (bottom left sensor)... ");
      bottomLeft.rangingTest(&bottomLeftMeasurement, false); // pass in 'true' to get debug data printout!
      if(bottomLeftMeasurement.RangeStatus != 4) {
        bottomLeftOutOfRange = false;
        bottomLeftRange = bottomLeftMeasurement.RangeMilliMeter;
        Serial.println(bottomLeftRange);
      }
      else {
        bottomLeftOutOfRange = true;
      }

      delay(100);
      
      tcaselect(1);
      Serial.println("Reading a measurement (top left sensor)... ");
      topLeft.rangingTest(&topLeftMeasurement, false); // pass in 'true' to get debug data printout!
      if(topLeftMeasurement.RangeStatus != 4) {
        topLeftOutOfRange = false;
        topLeftRange = topLeftMeasurement.RangeMilliMeter;
        Serial.println(topLeftRange);
        Serial.println("-------------");
      }
      else {
        topLeftOutOfRange = true;
      }

      if(!bottomLeftOutOfRange && !topLeftOutOfRange) {
          //correct right  
          if(topLeftRange < bottomLeftRange - normalThreshold) {
            if(topLeftRange < bottomLeftRange - superThreshold) {
              rightSpeed = 50;
              leftSpeed = 150;
            }
            else {
              rightSpeed = 50;
              leftSpeed = 100; 
            }
          }

          //correct left
          else if(topLeftRange > bottomLeftRange + normalThreshold) {
            if(topLeftRange > bottomLeftRange + superThreshold) {
              rightSpeed = 150;
              leftSpeed = 50;    
            }
            else {
              rightSpeed = 100;
              leftSpeed = 50;  
            } 
          }

          //go forward
          else {
            rightSpeed = 150;
            leftSpeed = 150;
          }

          M[0].run(FORWARD); //right motor
          M[1].run(FORWARD); //left motor
          M[0].Setpoint = rightSpeed;
          M[1].Setpoint = leftSpeed;
      }
      else {
        Serial.println("Out of Range. Stopping Algorithm.");
        for(int j=0;j<2;j++) {
          M[j].run(STOP);
          M[j].Setpoint = 0;
        }
        break;
      }
    }
    else {
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

/*double turnRight(int targetHeading) {
  M[0].resetPosition();
  M[1].resetPosition();
  Speed = 115;
  int flag = 0;
  
  delay(500);
  //Get initial heading from IMU
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("Starting function call\tInitial X: ");
  xValInitial = event.orientation.x;
  Serial.print(xValInitial, 4);
  Serial.print("\n");

  double diff = 0;
  
  //Setup timing
  runningMotors = false;
  previousTime = millis();
  
  while (1) {
    // Find the difference between the current heading and the initial heading
    diff = getDiff();
    
    // Turn the full angle
    if (abs(diff)<=targetHeading-10) {
      unsigned long currentTime = millis();
      //Toggle motors every 50 ms
      if (currentTime-previousTime>runTime) {
        //If motors were running turn them off
        if (runningMotors) {
          M[0].run(STOP);
          M[1].run(STOP);
          M[0].Setpoint = 0;
          M[1].Setpoint = 0;
          runningMotors = false;
          Serial.print("Current X: ");
          Serial.print(xVal, 4);
          Serial.print("\n");
          previousTime = millis();
        }
        //If motors are off turn them on
        else {
          Serial.print("Current X: ");
          Serial.print(xVal, 4);
          Serial.print("\n");
          M[0].run(BACKWARD);
          M[1].run(FORWARD);
          M[0].Setpoint = Speed;
          M[1].Setpoint = Speed;
          runningMotors = true;
          previousTime = millis();
        }
      }
    }
    //Once turned the angle, shutoff the motors
    else {
      M[0].run(STOP);
      M[1].run(STOP);
      M[0].Setpoint = 0;
      M[1].Setpoint = 0;
      break;
    }

    if((millis()-lastMilli) >= LOOPTIME) {
      lastMilli = millis();
      M[0].updatePID();
      M[1].updatePID();         
    }
  }
  delay(500);
  diff = getDiff();
  
  while(1) {
    Speed = 75;
    while (targetHeading-abs(diff)>0.5) {
      quickturnRight();
      diff = getDiff();
    }
    while (targetHeading-abs(diff)<-0.5) {
      quickturnLeft();
      diff = getDiff();
    }
    if (abs(targetHeading-abs(diff))<=0.5) {
      Serial.print("Current X: ");
      Serial.print(xVal, 4);
      Serial.print("\n");
      break;
    }
  }
  return (targetHeading-abs(diff));
}

double turnLeft(int targetHeading) {
  M[0].resetPosition();
  M[1].resetPosition();
  Speed = 115;
  int flag = 0;
  
  delay(500);
  //Get initial heading from IMU
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("Starting function call\tInitial X: ");
  xValInitial = event.orientation.x;
  Serial.print(xValInitial, 4);
  Serial.print("\n");
  
  double diff = 0;
  
  //Setup timing
  runningMotors = false;
  previousTime = millis();
  
  while (1) {
    // Find the difference between the current heading and the initial heading
    diff = getDiff();
    
    // Turn the full angle
    if (abs(diff)<=targetHeading-10) {
      unsigned long currentTime = millis();
      //Toggle motors every 50 ms
      if (currentTime-previousTime>runTime) {
        //If motors were running turn them off
        if (runningMotors) {
          M[0].run(STOP);
          M[1].run(STOP);
          M[0].Setpoint = 0;
          M[1].Setpoint = 0;
          runningMotors = false;
          Serial.print("Current X: ");
          Serial.print(xVal, 4);
          Serial.print("\n");
          previousTime = millis();
        }
        //If motors are off turn them on
        else {
          Serial.print("Current X: ");
          Serial.print(xVal, 4);
          Serial.print("\n");
          M[0].run(FORWARD);
          M[1].run(BACKWARD);
          M[0].Setpoint = Speed;
          M[1].Setpoint = Speed;
          runningMotors = true;
          previousTime = millis();
        }
      }
    }
    //Once turned the angle, shutoff the motors
    else {
      M[0].run(STOP);
      M[1].run(STOP);
      M[0].Setpoint = 0;
      M[1].Setpoint = 0;
      break;
    }

    if((millis()-lastMilli) >= LOOPTIME) {
      lastMilli = millis();
      M[0].updatePID();
      M[1].updatePID();         
    }
  }
  delay(500);
  diff = getDiff();
  
  while(1) {
    Speed = 75;
    while (targetHeading-abs(diff)>0.5) {
      quickturnLeft();
      diff = getDiff();
    }
    while (targetHeading-abs(diff)<-0.5) {
      quickturnRight();
      diff = getDiff();
    }
    if (abs(targetHeading-abs(diff))<=0.5) {
      break;
    }
  }
  return (targetHeading-abs(diff));
}

double getDiff() {
  sensors_event_t event;
  bno.getEvent(&event);
  xVal = event.orientation.x;
  double difference = xVal-xValInitial;
  //Correct the difference degrees to be less than 180
  if (difference > 180) {
   difference = -360 + difference;
  } 
  else if (difference < -180) {
   difference = 360 + difference;
  }
  return difference;
}

void quickturnRight() {
  M[0].run(BACKWARD);
  M[1].run(FORWARD);

  sensors_event_t event;

  M[0].setSpeed(75); 
  M[1].setSpeed(75);
  runningMotors = true;
  delay(50);

  M[0].run(STOP);
  M[1].run(STOP);
  M[0].setSpeed(0); 
  M[1].setSpeed(0);
  runningMotors = false;
  
  delay(500);
  bno.getEvent(&event);
  xVal = event.orientation.x;
  
}

void quickturnLeft() {
  M[0].run(FORWARD);
  M[1].run(BACKWARD);

  sensors_event_t event;

  M[0].setSpeed(75); 
  M[1].setSpeed(75);
  runningMotors = true;
  delay(50);

  M[0].run(STOP);
  M[1].run(STOP);
  M[0].setSpeed(0); 
  M[1].setSpeed(0);
  runningMotors = false;
  delay(500);
  bno.getEvent(&event);
  xVal = event.orientation.x; 
}*/
