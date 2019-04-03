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

Adafruit_VL53L0X backLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X frontLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X backRight = Adafruit_VL53L0X();
Adafruit_VL53L0X frontRight = Adafruit_VL53L0X();

/// void setup()
///
/// Author: UA IEEE SoutheastCon Team 2019
/// Description: Sets up and initializes the required values and objects
///				 to operate the robot.
void setup() {
	Serial.begin(9600, SERIAL_8N1);
	pinMode(STATUS_PIN, OUTPUT);
  Wire.begin();
	Speed = 100;

    Serial.println("Please.");
	delay(300);
	irSetup();
    Serial.println("Pretty please.");

    tcaselect(7);
    Serial.println("Pretty pretty please.");
    //Get initial heading from IMU
    bno.setExtCrystalUse(true);
    bno.getEvent(&heading);
    Serial.print("Initial X: ");
    Serial.print(heading.orientation.x, 4);
    Serial.print("\n");
    delay(500);
    bno.getEvent(&heading);
    //  Serial.print("Initial X: ");
    //  Serial.print(heading.orientation.x, 4);
    //  Serial.print("\n");



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

//	if (process_command) {
//		Serial.print("Command is ");
//		Serial.print(cmd);
//		Serial.print("\n");
//		Serial.print("Data is ");
//		Serial.print(data);
//		Serial.print("\n");
//	}

	// Update status pin for Raspberry Pi
	digitalWrite(STATUS_PIN, (process_command) ? LOW : HIGH);

	// Process command
	// (Command 1: Drive by distance)
	if (cmd == 1) {
//		Serial.print("Driving straight for ");
//		Serial.print(data);
//		Serial.print(" inches.");
//		Serial.print("\n");
		driveDistance(data); 
	}
 
	// (Command 2: Turn)
	else if(cmd == 2) {
//    Serial.print("Turning to position ");
//    Serial.print(data, 4);
//    Serial.print(".\n");
        turn(data);
	}
 
	// (Command 3: Drive by time)
	else if(cmd == 3) {
		driveTimed(data);
	}
 
	// (Command 4: Operate the Belt)
	else if(cmd == 4) {
//		Serial.print("Running belt action ");
		int beltOp = (int) data;
//		Serial.print(beltOp);
//		Serial.print(".\n");
    B.moveTo((BeltPosition) beltOp);
//		Serial.print("Belt position is ");
//		Serial.print(B.getPosition());
//		Serial.print(".\n");
 
	// (Command 5: Wall Follow [Right Side])
	} else if (cmd == 5) {
//		Serial.print("Following the Wall (Right Side) for ");
//		Serial.print(data);
//		Serial.print(" inches.");
//		Serial.print("\n");
		wallFollowRight(data);
    		
	// (Command 6: Wall Follow [Left Side])
	} else if (cmd == 6) {
//		Serial.print("Following the Wall (Left Side) for ");
//		Serial.print(data);
//		Serial.print(" inches.");
//		Serial.print("\n");
		wallFollowLeft(data);
   
	} else if (cmd == 7) {
      sendData();
	}

	// Reset command
	if (process_command) {
		cmd = 0;
		data = 0.0;
		process_command = false;
		//Serial.print("EOL\n");
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
//		Serial.print(" Millis: ");
//		Serial.print(millis());
//		Serial.print("\tstart: ");
//		Serial.print(startTime);
//		Serial.print("\tend: ");
//		Serial.print(endTime);
//		Serial.print("\tstart + end: ");
//		Serial.print(startTime + endTime);
//		Serial.print("\n");
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
  boolean backLeftOutOfRange;
  boolean frontLeftOutOfRange;
  int superThreshold = 30;
  int normalThreshold = 10;
  
  M[0].resetPosition();
  M[1].resetPosition();

  VL53L0X_RangingMeasurementData_t backLeftMeasurement;
  VL53L0X_RangingMeasurementData_t frontLeftMeasurement;

  uint16_t backLeftRange;
  uint16_t frontLeftRange;
  
  int rightSpeed = 150;
  int leftSpeed = 150;
  double tickGoal = inchesToTicks(distance);
  
  while(1) {
    double posRight = abs(M[0].getPosition());
    double posLeft = abs(M[1].getPosition());
    double posAvg = (posRight + posLeft) / 2.0;
    if(posAvg < tickGoal) {
//      Serial.println("-------------");
      tcaselect(3);
//      Serial.println("Reading a measurement (back left sensor)... ");
      backLeft.rangingTest(&backLeftMeasurement, false); // pass in 'true' to get debug data printout!
      if(backLeftMeasurement.RangeStatus != 4) {
        backLeftOutOfRange = false;
        backLeftRange = backLeftMeasurement.RangeMilliMeter;
//        Serial.println(backLeftRange);
      }
      else {
        backLeftOutOfRange = true;
      }

      delay(100);
      
      tcaselect(2);
//      Serial.println("Reading a measurement (front left sensor)... ");
      frontLeft.rangingTest(&frontLeftMeasurement, false); // pass in 'true' to get debug data printout!
      if(frontLeftMeasurement.RangeStatus != 4) {
        frontLeftOutOfRange = false;
        frontLeftRange = frontLeftMeasurement.RangeMilliMeter;
//        Serial.println(frontLeftRange);
//        Serial.println("-------------");
      }
      else {
        frontLeftOutOfRange = true;
      }

      if(!backLeftOutOfRange && !frontLeftOutOfRange) {
          //correct right  
          if(frontLeftRange < backLeftRange - normalThreshold) {
            if(frontLeftRange < backLeftRange - superThreshold) {
              rightSpeed = 150;
              leftSpeed = 200;
            }
            else {
              rightSpeed = 100;
              leftSpeed = 150; 
            }
          }

          //correct left
          else if(frontLeftRange > backLeftRange + normalThreshold) {
            if(frontLeftRange > backLeftRange + superThreshold) {
              rightSpeed = 200;
              leftSpeed = 100;    
            }
            else {
              rightSpeed = 150;
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
//          M[0].setSpeed(rightSpeed);
//          M[1].setSpeed(leftSpeed);
      }
      else {
//        Serial.println("Out of Range. Stopping Algorithm.");
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

    if((millis()-lastMilli) >= LOOPTIME) {
      lastMilli = millis();
      M[0].updatePID();
      M[1].updatePID();         
    }  
  }  
}

void wallFollowRight(int distance) {
  boolean backRightOutOfRange;
  boolean frontRightOutOfRange;
  int superThreshold = 30;
  int normalThreshold = 10;
  
  M[0].resetPosition();
  M[1].resetPosition();

  VL53L0X_RangingMeasurementData_t backRightMeasurement;
  VL53L0X_RangingMeasurementData_t frontRightMeasurement;

  uint16_t backRightRange;
  uint16_t frontRightRange;
  
  int rightSpeed = 150;
  int leftSpeed = 150;
  double tickGoal = inchesToTicks(distance);
  
  while(1) {
    double posRight = abs(M[0].getPosition());
    double posLeft = abs(M[1].getPosition());
    double posAvg = (posRight + posLeft) / 2.0;
    if(posAvg < tickGoal) {
//      Serial.println("-------------");
      
      tcaselect(0);
//      Serial.println("Reading a measurement (back right sensor)... ");
      backRight.rangingTest(&backRightMeasurement, false); // pass in 'true' to get debug data printout!
      if(backRightMeasurement.RangeStatus != 4) {
        backRightOutOfRange = false;
        backRightRange = backRightMeasurement.RangeMilliMeter;
//        Serial.println(backRightRange);
      }
      else {
        backRightOutOfRange = true;
      }

      delay(100);
      
      tcaselect(1);
//      Serial.println("Reading a measurement (front right sensor)... ");
      frontRight.rangingTest(&frontRightMeasurement, false); // pass in 'true' to get debug data printout!
      if(frontRightMeasurement.RangeStatus != 4) {
        frontRightOutOfRange = false;
        frontRightRange = frontRightMeasurement.RangeMilliMeter;
//        Serial.println(frontRightRange);
//        Serial.println("-------------");
      }
      else {
        frontRightOutOfRange = true;
      }

      if(!backRightOutOfRange && !frontRightOutOfRange) {
          //correct right  
          if(frontRightRange < backRightRange - normalThreshold) {
            if(frontRightRange < backRightRange - superThreshold) {
              rightSpeed = 200;
              leftSpeed = 100;
            }
            else {
              rightSpeed = 150;
              leftSpeed = 100; 
            }
          }

          //correct left
          else if(frontRightRange > backRightRange + normalThreshold) {
            if(frontRightRange > backRightRange + superThreshold) {
              rightSpeed = 100;
              leftSpeed = 200;    
            }
            else {
              rightSpeed = 100;
              leftSpeed = 150;  
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
//        Serial.println("Out of Range. Stopping Algorithm.");
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
//    Serial.print("Beginning to turn. Desired change in heading is ");
//    Serial.print(abs(diff), 4);
//    Serial.print((diff < 0) ? "degrees left.\n" : "degrees right.\n");

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
//    Serial.print("Finished turning. New heading is ");
//    Serial.print(heading.orientation.x, 4);
//    Serial.print(" degrees (");
//    Serial.print(heading.orientation.x - target_heading, 4);
//    Serial.print(" degrees from target).\n");
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
  tcaselect(7);
  bno.getEvent(&heading);
//  Serial.print("heading: ");
//  Serial.println(heading.orientation.x);
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
    //Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
    
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }

  tcaselect(0);
  if (!backRight.begin()) {
    Serial.println(F("Failed to boot back right VL53L0X"));
    while(1);
  }

  tcaselect(1);
  if (!frontRight.begin()) {
    Serial.println(F("Failed to boot front right VL53L0X"));
    while(1);
  }

    tcaselect(2);
  if (!frontLeft.begin()) {
    Serial.println(F("Failed to boot front left VL53L0X"));
    while(1);
  }

  tcaselect(3);
  if (!backLeft.begin()) {
    Serial.println(F("Failed to boot back left VL53L0X"));
    while(1);
  }

  // Setup the IMU
  tcaselect(7);
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
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

double calculateWallPositioning(uint16_t back, uint16_t front) {
  double a;
  double b;
  //distance between IR's in inches
  double d = 3.25;

  if(back > front){
     a = (double) front;
     b = (double) back;
  }
  else {
    a = (double) back;
    b = (double) front;
  }

  double theta = atan2 (b - a, d);
  return theta * RAD_TO_DEG;
}

void sendData() {
  //heading
  tcaselect(7);
  bno.getEvent(&heading);
  Serial.print(heading.orientation.x, 4);
  Serial.print("\n");
  //ir0
  Serial.print(getMeasurement(backRight, 0));
  Serial.print("\n");
  //ir1
  Serial.print(getMeasurement(frontRight, 1));
  Serial.print("\n");
  //ir2
  Serial.print(getMeasurement(frontLeft, 2));
  Serial.print("\n");
  //ir3
  Serial.print(getMeasurement(backLeft, 3));
  Serial.print("\n");
  //ir4 DNE
  Serial.print("0");
  Serial.print("\n");
  //ir5 DNE
  Serial.print("0");
  Serial.print("\n");
  //ir6 DNE
  Serial.print("0");
  Serial.print("\n");
  Serial.print("0");
  Serial.print("\n");
}
