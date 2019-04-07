#include "PID_v1.h"
#include "Encoder.h"
#include "Motors.h"
#include <Wire.h>
#include <math.h> 
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Belt.h"
extern "C" { 
  #include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include "Adafruit_VL53L0X.h"

// Status pin for Raspberry Pi
#define STATUS_PIN 36
//Address of I2C Mux
#define TCAADDR 0x70

int Speed = 0;
unsigned long lastMilli = 0;

int cmd = 0;
double data = 0;

//Set the delay between samples of the IMU
#define BNO055_SAMPLERATE_DELAY_MS (10)

//Set the delay between samples of the IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t heading;

// Motor(MotorPWM,MotorIn1,MotorIn2,EncoderA,EncoderB)
Motor M[2] = 
{ 
	{9,6,7,2,4}, // Right motor M[0]
	{10,33,35,3,5} // left motor M[1]
};

// Belt(PWM, In1, In2, EncoderA, EncoderB)
Belt B = {8,32,34,22,18};

Adafruit_VL53L0X bottomLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X topLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X bottomRight = Adafruit_VL53L0X();
Adafruit_VL53L0X topRight = Adafruit_VL53L0X();

/// void setup()
///
/// Author: UA IEEE SoutheastCon Team 2019
/// Description: Sets up and initializes the required values and objects
///				 to operate the robot.
void setup() {
	Serial.begin(9600, SERIAL_8N1);
	pinMode(STATUS_PIN, OUTPUT);
	Speed = 100;

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
	bno.getEvent(&heading);
	Serial.print("Initial X: ");
	Serial.print(heading.orientation.x, 4);
	Serial.print("\n");
	delay(500);
	bno.getEvent(&heading);
	Serial.print("Initial X: ");
	Serial.print(heading.orientation.x, 4);
	Serial.print("\n");

	irSetup();

	// Status is HIGH when ready for commands, and LOW when processing commands
	digitalWrite(STATUS_PIN, HIGH);
}

/// void loop()
///
/// Author: UA IEEE SoutheastCon Team 2019
/// Description: Repeatedly executes. Waits for a command from the Raspberry Pi or
/// 			 Serial Monitor.
void loop() {
	// Flag to identify when a command is being processed
	static bool process_command = false;

	// Read command
	while (Serial.available() > 0) {
		cmd = Serial.parseInt();
		data = (double) Serial.parseFloat();
		// Throw away the terminating newline character
		Serial.read();
		// We are about to process a command
		if (!process_command) process_command = true;
	}

	if (process_command) {
		Serial.print("Command is ");
		Serial.print(cmd);
		Serial.print("\n");
		Serial.print("Data is ");
		Serial.print(data);
		Serial.print("\n");
	}

	// Update status pin for Raspberry Pi
	digitalWrite(STATUS_PIN, (process_command) ? LOW : HIGH);

	// Process command
	// (Command 1: Drive by distance)
	if (cmd == 1) {
		Serial.print("Driving straight for ");
		Serial.print(data);
		Serial.print(" inches.");
		Serial.print("\n");
		driveDistance(data); 
	}
 
	// (Command 2: Turn)
	else if(cmd == 2) {
    Serial.print("Turning to position ");
    Serial.print(data, 4);
    Serial.print(".\n");
    turn(data);
	}
 
	// (Command 3: Drive by time)
	else if(cmd == 3) {
		Serial.print("Driving straight for ");
		Serial.print(data);
		Serial.print(" milliseconds.");
		Serial.print("\n");
		driveTimed(data);
	}
 
	// (Command 4: Operate the Belt)
	else if(cmd == 4) {
		Serial.print("Running belt action ");
		int beltOp = (int) data;
		Serial.print(beltOp);
		Serial.print(".\n");
		B.moveTo((BeltPosition) beltOp);
		Serial.print("Belt position is ");
		Serial.print(B.getPosition());
		Serial.print(".\n");
    
	// (Command 5: Wall Follow [Right Side])
	} else if (cmd == 5) {
		Serial.print("Following the Wall (Right Side) for ");
		Serial.print(data);
		Serial.print(" inches.");
		Serial.print("\n");
		wallFollowRight(data);
    		
	// (Command 6: Wall Follow [Left Side])
	} else if (cmd == 6) {
		Serial.print("Following the Wall (Left Side) for ");
		Serial.print(data);
		Serial.print(" inches.");
		Serial.print("\n");
		wallFollowLeft(data);
	}

	// Reset command
	if (process_command) {
		cmd = 0;
		data = 0.0;
		process_command = false;
		Serial.print("EOL\n");
	}
}

/// void driveTimed(int endTime)
///
/// Author: Katie McCray
/// Description: Sets the motors to drive a given amount of time in milliseconds.
/// Parameter:
/// @int endTime - The amount of time to drive for, in milliseconds.
void driveTimed(int endTime) {
	unsigned long startTime = millis();
	while(1) {
		Serial.print(" Millis: ");
		Serial.print(millis());
		Serial.print("\tstart: ");
		Serial.print(startTime);
		Serial.print("\tend: ");
		Serial.print(endTime);
		Serial.print("\tstart + end: ");
		Serial.print(startTime + endTime);
		Serial.print("\n");
		if(millis() < startTime + endTime) {
			M[0].run(FORWARD); //right motor
			M[1].run(BACKWARD); //left motor
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

/// void driveDistance(int distance)
///
/// Author: Katie McCray
/// Description: Sets the motors to drive a given distance in inches.
/// Parameter:
/// @int distance - The distance to drive for, in inches.
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
      tcaselect(3);
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
      
      tcaselect(2);
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
              rightSpeed = 100;
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
          //M[0].Setpoint = rightSpeed;
          //M[1].Setpoint = leftSpeed;
          M[0].setSpeed(rightSpeed);
          M[1].setSpeed(leftSpeed);
      }
      else {
        Serial.println("Out of Range. Stopping Algorithm.");
        for(int j=0;j<2;j++) {
          M[j].run(STOP);
          //M[j].Setpoint = 0;
          M[j].setSpeed(0);
        }
        break;
      }
    }
    else {
      for(int j=0;j<2;j++) {
        M[j].run(STOP);
        //M[j].Setpoint = 0;
        M[j].setSpeed(0);
      }
      break;
    }
    
//    if((millis()-lastMilli) >= LOOPTIME) {
//      lastMilli = millis();
//      M[0].updatePID();
//      M[1].updatePID();         
//    }  
  }  
}

void wallFollowRight(int distance) {
  boolean bottomRightOutOfRange;
  boolean topRightOutOfRange;
  int superThreshold = 30;
  int normalThreshold = 10;
  
  M[0].resetPosition();
  M[1].resetPosition();

  VL53L0X_RangingMeasurementData_t bottomRightMeasurement;
  VL53L0X_RangingMeasurementData_t topRightMeasurement;

  uint16_t bottomRightRange;
  uint16_t topRightRange;
  
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
      Serial.println("Reading a measurement (bottom right sensor)... ");
      bottomRight.rangingTest(&bottomRightMeasurement, false); // pass in 'true' to get debug data printout!
      if(bottomRightMeasurement.RangeStatus != 4) {
        bottomRightOutOfRange = false;
        bottomRightRange = bottomRightMeasurement.RangeMilliMeter;
        Serial.println(bottomRightRange);
      }
      else {
        bottomRightOutOfRange = true;
      }

      delay(100);
      
      tcaselect(1);
      Serial.println("Reading a measurement (top right sensor)... ");
      topRight.rangingTest(&topRightMeasurement, false); // pass in 'true' to get debug data printout!
      if(topRightMeasurement.RangeStatus != 4) {
        topRightOutOfRange = false;
        topRightRange = topRightMeasurement.RangeMilliMeter;
        Serial.println(topRightRange);
        Serial.println("-------------");
      }
      else {
        topRightOutOfRange = true;
      }

      if(!bottomRightOutOfRange && !topRightOutOfRange) {
          //correct right  
          if(topRightRange < bottomRightRange - normalThreshold) {
            if(topRightRange < bottomRightRange - superThreshold) {
              rightSpeed = 150;
              leftSpeed = 50;
            }
            else {
              rightSpeed = 100;
              leftSpeed = 50; 
            }
          }

          //correct left
          else if(topRightRange > bottomRightRange + normalThreshold) {
            if(topRightRange > bottomRightRange + superThreshold) {
              rightSpeed = 50;
              leftSpeed = 150;    
            }
            else {
              rightSpeed = 50;
              leftSpeed = 100;  
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

//void rightParallelToWall() {
//  
//}
//
//void leftParallelToWall() {
//  boolean bottomLeftOutOfRange;
//  boolean topLeftOutOfRange;
//  int superThreshold = 30;
//  int normalThreshold = 10;
//  uint16_t bottomLeftRange;
//  uint16_t topLeftRange;
//  double wallAngle;
//
//  //initial position to wall
//  bottomLeftRange = getMeasurement(bottomLeft, 0);
//  if(bottomLeftRange == -1) {
//    bottomLeftOutOfRange = true;
//  }
//  else{
//    bottomLeftOutOfRange = false;
//  }
//  delay(100);
//
//  topLeftRange = (topLeft, 1);
//  if(topLeftRange == -1) {
//    topLeftOutOfRange = true;
//  }
//  else{
//    topLeftOutOfRange = false;
//  }
//  delay(100);
//  if(!bottomLeftOutOfRange && !topLeftOutOfRange) {
//    wallAngle = calculateWallPositioning(bottomLeftRange, topLeftRange);
//  }
//  
//  while(wallAngle < -10 && wallAngle > 10) {
//    bottomLeftRange = getMeasurement(bottomLeft, 0);
//    if(bottomLeftRange == -1) {
//      bottomLeftOutOfRange = true;
//    }
//    else{
//      bottomLeftOutOfRange = false;
//    }
//    delay(100);
//
//    topLeftRange = (topLeft, 1);
//    if(topLeftRange == -1) {
//      topLeftOutOfRange = true;
//    }
//    else{
//      topLeftOutOfRange = false;
//    }
//    delay(100);
//
//    if(
//  }
//   
//}

/// double inchesToTicks(int inches)
///
/// Author: Katie McCray
/// Description: Converts inches to encoder ticks.
/// Parameter:
/// @int inches - The value (in inches) to convert.
/// Return: (double) Encoder ticks to travel the provided amount of inches (int inches).
double inchesToTicks(int inches) {
	double ticksPerInch = 2700.0 / CIRCUMFERENCE;
	return inches * ticksPerInch;
}

void turn(double target_heading) {
    M[0].resetPosition();
    M[1].resetPosition();    

    // Calculate desired change in heading and motor directions
    double diff = getDiff(target_heading);
    int dir[2];
    if (diff < 0) {
        dir[0] = FORWARD;
        dir[1] = BACKWARD;
    } else {
        dir[0] = BACKWARD;
        dir[1] = FORWARD;
    }
    
    // Indicate to user the intended action
    Serial.print("Beginning to turn. Desired change in heading is ");
    Serial.print(abs(diff), 4);
    Serial.print((diff < 0) ? "degrees left.\n" : "degrees right.\n");

    // Quickly turn to target heading until within 10 degrees
    Speed = 50;
    while ((diff < 0) ? diff < -10 : diff > 10) {
        M[0].run(dir[0]);
        M[0].Setpoint = Speed;
        M[1].run(dir[1]);
        M[1].Setpoint = Speed;

        if((millis()-lastMilli) >= LOOPTIME) {
          lastMilli = millis();
          M[0].updatePID();
          M[1].updatePID();        
        }
        
        diff = getDiff(target_heading);
    }
    stopWheels();

    // Update current heading, difference in heading, and direction
    diff = getDiff(target_heading);
    if (diff < 0) {
        dir[0] = FORWARD;
        dir[1] = BACKWARD;
    } else {
        dir[0] = BACKWARD;
        dir[1] = FORWARD;
    }

    // Slowly move to target heading until within 0.5 degreses
    Speed = 75;
    while (abs(diff) > 0.125) {
        // Start turning
        M[0].run(dir[0]);
        M[0].setSpeed(Speed);
        M[1].run(dir[1]);
        M[1].setSpeed(Speed);
        delay(50);
        stopWheels();
        diff = getDiff(target_heading);
        if (diff < 0) {
            dir[0] = FORWARD;
            dir[1] = BACKWARD;
        } else {
            dir[0] = BACKWARD;
            dir[1] = FORWARD;
        }
    }

    // Indicate to user the performed action
    Serial.print("Finished turning. New heading is ");
    Serial.print(heading.orientation.x, 4);
    Serial.print(" degrees (");
    Serial.print(heading.orientation.x - target_heading, 4);
    Serial.print(" degrees from target).\n");
}

void stopWheels() {
    M[0].run(STOP);
    M[0].setSpeed(0);
    M[1].run(STOP);
    M[1].setSpeed(0);

    long time_start = millis();
    long M0_start = M[0].getPosition();
    long M1_start = M[1].getPosition();
    while (millis() - time_start < 50) {
        if (M0_start - M[0].getPosition() != 0
            || M1_start - M[1].getPosition() != 0)
        {
            M0_start = M[0].getPosition();
            M1_start = M[1].getPosition();
            time_start = millis();
        }
    }
}

double getDiff(double target_heading) {
  bno.getEvent(&heading);
  double output = target_heading - heading.orientation.x;
  if (output > 180) {
    output -= 360;
  } else if (output < -180) {
    output += 360;
  }
  return output;
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void irSetup() {
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

    tcaselect(2);
  if (!topRight.begin()) {
    Serial.println(F("Failed to boot top right VL53L0X"));
    while(1);
  }

  tcaselect(3);
  if (!bottomRight.begin()) {
    Serial.println(F("Failed to boot bottom right VL53L0X"));
    while(1);
  }
}

uint16_t getMeasurement(Adafruit_VL53L0X irSensor, uint8_t channel) {
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t range;
  
  tcaselect(channel);
  
  irSensor.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if(measure.RangeStatus != 4) {
    range = measure.RangeMilliMeter;
    return range;
  }
  else {
    return -1; 
  }  
}

double calculateWallPositioning(uint16_t bottom, uint16_t top) {
  double a;
  double b;
  //distance between IR's in inches
  double d = 3.25;

  if(bottom > top){
     a = (double) top;
     b = (double) bottom;
  }
  else {
    a = (double) bottom;
    b = (double) top;
  }

  double theta = atan2 (b - a, d);
  return theta * RAD_TO_DEG;
}
