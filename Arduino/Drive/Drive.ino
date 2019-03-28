#include "PID_v1.h"
#include "Encoder.h"
#include "Motors.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "Belt.h"
#include "WallFollow/IRSensor.cpp"
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
#define VL53SETUP 0x03
// Maximum amount of sensors allowed.
const int SIZE = 8;

//Set the delay between samples of the IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
double xVal;
double xValInitial;

//Setup timing variables
unsigned long previousTime = 0;
//Run for 50 ms and then off for 50 ms
int runTime = 50;
bool runningMotors;

//Motor(MotorPWM,MotorIn1,MotorIn2,EncoderA,EncoderB)
Motor M[2] = 
{ 
  {9,6,7,2,4}, // right motor
  {10,33,35,3,5} // left motor
};

Belt B = {8,32,34,22,18};

// Sensors and WallFollow Algorithm
IRSensor** sensors;
WallFollow* wallAlg;

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
  wallAlg->Initialize(1, 0, 100);

  // Status is HIGH when ready for commands, and LOW when processing commands
  digitalWrite(STATUS_PIN, HIGH);
}

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
  if (cmd == 1) {
    Serial.print("Driving straight for ");
    Serial.print(data);
    Serial.print(" inches.");
    Serial.print("\n");
    driveDistance(data); 
  }

  else if(cmd == 2) {
    Serial.print("Turning right for ");
    Serial.print(data);
    Serial.print(" degrees.");
    Serial.print("\n");
    turnRight(data);
  }

  else if(cmd == 3) {
    Serial.print("Turning left for ");
    Serial.print(data);
    Serial.print(" degrees.");
    Serial.print("\n");
    turnLeft(data);
  }

  else if(cmd == 4) {
    Serial.print("Driving straight for ");
    Serial.print(data);
    Serial.print(" milliseconds.");
    Serial.print("\n");
    driveTimed(data);
  }

  else if(cmd == 5) {
    Serial.print("Running belt action ");
    int beltOp = (int) data;
    Serial.print(beltOp);
    Serial.print(".\n");
    B.moveTo((BeltPosition) beltOp);
    Serial.print("Belt position is ");
    Serial.print(B.getPosition());
    Serial.print(".\n");
  }

  // Reset command
  if (process_command) {
    cmd = 0;
    data = 0.0;
    process_command = false;
    Serial.print("EOL\n");
  }
}

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

void driveDistance(int distance) {
  M[0].resetPosition();
  M[1].resetPosition();
  Speed = 200;
  double tickGoal = inchesToTicks(distance);
  while(1) {
    double posRight = abs(M[0].getPosition());
    double posLeft = abs(M[1].getPosition());
    double posAvg = (posRight + posLeft) / 2.0;
    
    wallAlg->Act();
    
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

double inchesToTicks(int inches) {
  double ticksPerInch = 2700.0 / CIRCUMFERENCE;
  return inches * ticksPerInch;
}

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
  char buf[100];
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
