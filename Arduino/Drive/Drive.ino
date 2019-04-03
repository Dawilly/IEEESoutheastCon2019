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
#define STATUS_PIN 30

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
double xVal;
double xValInitial;

//Setup timing variables
unsigned long previousTime = 0;
// Run for 50 ms and then off for 50 ms
int runTime = 50;
bool runningMotors;

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
		turnRight(data);
	}
	// (Command 3: Turn left)
	else if(cmd == 3) {
		Serial.print("Turning left for ");
		Serial.print(data);
		Serial.print(" degrees.");
		Serial.print("\n");
		turnLeft(data);
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
void driveWallFollow(int distance) {
	bool running = true;
	double tickGoal = inchesToTicks(distance);
	double posRight = 0.0;
	double posLeft = 0.0;
	double posAvg = 0.0;
	double xVal = 0.0;
	int adjustment = 0;
	
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

/// double turnRight(int targetHeading)
///
/// Author: Brandon Quinn
/// Description: Drives the motors to perform a right turn with a given angle.
/// Parameter:
/// @int targetHeading - The angle to turn by (in degrees)
/// Returns: (double) The difference between targetHeading and how far it turned.
double turnRight(int targetHeading) {
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

/// turnLeft(int targetHeading)
///
/// Author: Brandon Quinn
/// Description: Drives the motors to perform a left turn with a given angle.
/// Parameter:
/// @int targetHeading - The angle to turn by (in degrees)
/// Returns: (double) The difference between targetHeading and how far it turned.
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

/// double getDiff()
///
/// Author: Brandon Quinn
/// Description: Gets the difference between xVal and xValInitial, with corrections.
/// Returns: (double) The difference between xVal and xValInitial.
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

/// void quickturnRight()
///
/// Author: Brandon Quinn
/// Description: Uses the motors to perform a quick turn to the right.
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

/// void quickturnLeft()
///
/// Author: Brandon Quinn
/// Description: Uses the motors to perform a quick turn to the left.
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
