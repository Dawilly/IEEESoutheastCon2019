#include "PID_v1.h"
#include "Encoder.h"
#include "Motors.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Belt.h"
//#include "IRSensor/IRSensor.cpp"
#include "WallFollow/WallFollow.cpp"

// Status pin for Raspberry Pi
#define STATUS_PIN 36

int Speed = 0;
unsigned long lastMilli = 0;

int cmd = 0;
double data = 0;

//Set the delay between samples of the IMU
#define BNO055_SAMPLERATE_DELAY_MS (10)

// Sensors configuration (Which sensors exist at which input of the mux)
// Example: 0x13 = 0001 0011. Sensors at SD4/SC4, SD0/SC0 and SD1/SC1
// 0xC0 = 1100 0000
// 0x0C = 0000 1100
// 0xCF = 1100 1111
#define VL53SETUP 0xCF
// Maximum amount of sensors allowed.
const int SIZE = 8;

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

// Sensors and WallFollow Algorithm
IRSensor** sensors;
WallFollow* wallAlg;

/// void setup()
///
/// Author: UA IEEE SoutheastCon Team 2019
/// Description: Sets up and initializes the required values and objects
///				 to operate the robot.
void setup() {
	Serial.begin(9600, SERIAL_8N1);
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
	bno.getEvent(&heading);
	Serial.print("Initial X: ");
	Serial.print(heading.orientation.x, 4);
	Serial.print("\n");
	delay(500);
	bno.getEvent(&heading);
	Serial.print("Initial X: ");
	Serial.print(heading.orientation.x, 4);
	Serial.print("\n");

	// Setup the IR Sensors
	setupSensors(VL53SETUP);

	// Setup the Algorithm(s)
	wallAlg = new WallFollow(sensors, true);
	wallAlg->Initialize(LeftBottom, LeftTop, 25);

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
	// (Command 2: Turn Right)
	else if(cmd == 2) {
		Serial.print("Turning right for ");
		Serial.print(data);
		Serial.print(" degrees.");
		Serial.print("\n");
		bno.getEvent(&heading);
    turn((int)(heading.orientation.x + data) % 360);  
	}
	// (Command 3: Turn left)
	else if(cmd == 3) {
		Serial.print("Turning left for ");
		Serial.print(data);
		Serial.print(" degrees.");
		Serial.print("\n");
		bno.getEvent(&heading);
    turn((int)(heading.orientation.x - data) % 360);
	}
	// (Command 4: Drive by time)
	else if(cmd == 4) {
		Serial.print("Driving straight for ");
		Serial.print(data);
		Serial.print(" milliseconds.");
		Serial.print("\n");
		driveTimed(data);
	}
	// (Command 5: Operate the Belt)
	else if(cmd == 5) {
		Serial.print("Running belt action ");
		int beltOp = (int) data;
		Serial.print(beltOp);
		Serial.print(".\n");
		B.moveTo((BeltPosition) beltOp);
		Serial.print("Belt position is ");
		Serial.print(B.getPosition());
		Serial.print(".\n");
	// (Command 6: Wall Follow [Right Side])
	} else if (cmd == 6) {
		Serial.print("Following the Wall (Right Side) for ");
		Serial.print(data);
		Serial.print(" inches.");
		Serial.print("\n");
		wallAlg->SetSensorsToTrace(RightBottom, RightTop);
		driveWallFollow(data);		
	// (Command 7: Wall Follow [Left Side])
	} else if (cmd == 7) {
		Serial.print("Following the Wall (Left Side) for ");
		Serial.print(data);
		Serial.print(" inches.");
		Serial.print("\n");
		wallAlg->SetSensorsToTrace(LeftBottom, LeftTop);
		driveWallFollow(data);
	}
  // Command 8: Turn to specified absolute heading
  else if (cmd == 8) {
    Serial.print("Turning to position ");
    Serial.print(data, 4);
    Serial.print(".\n");
    turn(data);
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
	Speed = 200;
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

/// void driveWallFollow(int distance)
///
/// Author: David Weil
/// Description: Sets the motors to drive, following the wall of the arena
///							with a given distance.
/// Parameters:
/// @int distance - The distance to follow the wall, in inches.

/// To-do: Implement initial turn.
void driveWallFollow(int distance) {
	bool running = true;
	double tickGoal = inchesToTicks(distance);
	double posRight = 0.0;
	double posLeft = 0.0;
	double posAvg = 0.0;
	int adjustment = 0;
	bool orientation = wallAlg->FollowingLeftOrRight();
	
	M[0].resetPosition();
	M[1].resetPosition();

	Speed = 100;
	double SpeedLeft = Speed;
	double SpeedRight = Speed;
	
	while(running) {
		posRight = abs(M[0].getPosition());
		posLeft = abs(M[1].getPosition());
		posAvg = (posRight + posLeft) / 2.0;
		
		adjustment = wallAlg->Act();
		
		if(posAvg < tickGoal) {
			switch(adjustment) {
				//Stride Left
				case 1:
					SpeedLeft = (Speed	* 0.75);
					SpeedRight = Speed;
					break;
				//Stride Right
				case 2:
					SpeedLeft = Speed;
					SpeedRight = (Speed * 0.75);
					break;
				//Forward
				case 0:
				default:
					SpeedLeft = Speed;
					SpeedRight = Speed;
					break;
			}
			M[0].run(FORWARD); //right motor
			M[1].run(FORWARD); //left motor
			M[0].Setpoint = SpeedRight;
			M[1].Setpoint = SpeedLeft;
		} else {
			//Unrolled loop for optimization. 
			M[0].run(STOP);
			M[0].Setpoint = 0;
			M[1].run(STOP);
			M[1].Setpoint = 0;
			running = false;
			continue;
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

/// setupSensors(uint8_t value)
///
/// Author: David Weil
/// Description: Allocates and initalizes the VL53L0X sensors and Measurement objects.
/// Parameter:
/// @uint8_t value - Informs which sensors exist at which pins of the Multiplex.
///					 See VL53SETUP defintion for detail and example.	
///
/// To-Do: Refactor to a SensorManager class.
void setupSensors(uint8_t value) {
	int i;
	int _bit;
	char buf[100];
	sensors = new IRSensor*[SIZE];

	for (i = 0; i < SIZE; i++) {
			_bit = value % 2;
			value = value / 2;
			TCASELECT(i);
			sensors[i] = (_bit == 1) ? new IRSensor(false) : NULL;
			
		if (i == RightBottom) {
			// No offset
		} else if (i == RightTop) {
			// No offset
		}
		
		if (sensors[i] != NULL && !sensors[i]->success) {
			// If the sensor failed to begin. Print the error message. Then delete the instance.
			sprintf(buf, "Failed to boot VL53L0X #%d.\n", i);
			Serial.println(buf);
			delete sensors[i];
		}
	}
	return;
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

/// TCASELECT(uint8_t pin)
///
/// Author: David Weil
/// Description: Sets the I2C Multiplexer (TCA9548A) to change the selected channel.
/// Parameter:
/// @uint8_t pin - The channel to change the Multiplexer to. Valid pins are [0 to 7].
///
/// To-Do: Refactor to a SensorManager class or Multiplexer class.
void TCASELECT(uint8_t pin) {
	if (pin > 7	|| pin < 0) return;
	Wire.beginTransmission(0x70);
	Wire.write(1 << pin);
	Wire.endTransmission();
}
