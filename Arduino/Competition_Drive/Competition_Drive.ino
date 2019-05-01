#include "PID_v1.h"
#include "Encoder.h"
#include "Motors.h"
#include "Belt.h"
#include <Wire.h>
#include <math.h> 
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
extern "C" { 
  #include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include "Adafruit_VL53L0X.h"
#include "Adafruit_PWMServoDriver.h"

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define STATUS_PIN 36
#define TCAADDR 0x70

#define TICKS_PER_INCH 429.718346348  // 2700.0 / CIRCUMFERENCE
#define INCHES_PER_TICK 0.00232710566 // CIRCUMFERENCE / 2700

#define SERVOMIN 120
#define SERVOMAX 600

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Instantiate an IMU and event structure
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t heading;

// Instantiate two motors, one for each wheel
//  M[0] is the right wheel
//  M[1] is the left wheel
Motor M[2] = {{9,6,7,2,4}, {10,33,35,3,5}};

// Instantiate the belt motor
Belt B = {8,32,34,22,18};

// Instantiate four IR sensors
//  IR[0] is back right
//  IR[1] is front right
//  IR[2] is front left
//  IR[3] if back left
Adafruit_VL53L0X IR[8] = {{}, {}, {}, {}, {}, {}, {}, {}};

// Function prototypes (only those with default parameters)
void driveDistanceIter(int &command, double data, double rpm_M0 = 100.0,
    double rpm_M1 = 100.0);

// Flag to indicate if a command call has been initialized
bool initialized = false;

// Values indicating changed distance with respect to the Pi's x and y coordinate system.
//  Encoder ticks are used to determine these values when driving straight
double x_delta = 0.0, y_delta = 0.0;

void setup() {
  Serial.begin(9600, SERIAL_8N1);
  Wire.begin();
  pinMode(STATUS_PIN, OUTPUT);

  i2cSetup();

  // Lower the flag
  uint16_t pulselen = map(50, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(3, 0, pulselen);

  // Status pin is HIGH when there are no commands to process, LOW otherwise
  digitalWrite(STATUS_PIN, HIGH);
}

void loop() {
  static int previous_command = 0;
  static int command = 0;
  static double data = 0.0;

  // Update command and data if necessary
  while (Serial.available() > 0) {
    command = Serial.parseInt();
    data = (double) Serial.parseFloat();
    Serial.read();
  }

  // When the command has changed, do the following BEFORE executing it:
  //  1) Stop the wheels
  //  2) Cache the previous command
  //  3) Inidicate the next command call is not intialized
  if (previous_command != command) {
    stopWheels();
    previous_command = command;
    initialized = false;
  }

  // Update the status pin for the Raspberry Pi
  digitalWrite(STATUS_PIN, (command == 0) ? HIGH : LOW);

  // Drive straight by distance
  if (command == 1) {
    driveDistanceIter(command, data);
  }

  // Turn to IMU heading
  else if (command == 2) {
    turn(data);
    command = 0;
  }

  // Drive straight by time
  else if (command == 3) {
    driveTimeIter(command, data);
  }

  // Operate the belt
  else if (command == 4) {
    B.moveTo((BeltPosition) ((int) data));
    command = 0;
  }

  // Wall follow with right side
  else if (command == 5) {
    wallFollowRightIter(command, data);
  }

  // Wall follow with left side
  else if (command == 6) {
    wallFollowLeftIter(command, data);
  }

  // Send the Raspberry Pi IMU heading and IR readings
  else if (command == 7) {
    sendData();
    command = 0;
  }

  //raise the flag
  else if (command == 8) {
    raiseTheFlag();
  }

}

// Command 1 iterator: Drive straight for distance
void driveDistanceIter(int &command, double distance, double rpm_M0,
    double rpm_M1)
  {
  static unsigned long cached_time = 0;
  static double tick_goal = 0.0;
  static double previous_tick_average = 0.0;
  static uint8_t dir = STOP;
  if (!initialized) {
    M[0].resetPosition();
    M[1].resetPosition();
    cached_time = millis();
    tick_goal = abs(distance) * TICKS_PER_INCH;
    previous_tick_average = 0;
    dir = (distance > 0.0) ? FORWARD : BACKWARD;
    initialized = true;
  }
  
  double tick_average = (abs(M[0].getPosition()) + abs(M[1].getPosition())) / 2.0;
  if (tick_average < tick_goal) {
    M[0].run(dir);
    M[0].Setpoint = rpm_M0;
    M[1].run(dir);
    M[1].Setpoint = rpm_M1;
  }
  else {
    command = 0;
    stopWheels();
    tick_average = (abs(M[0].getPosition()) + abs(M[1].getPosition())) / 2.0;
  }

  if (millis() - cached_time >= LOOPTIME) {
    cached_time = millis();
    M[0].updatePID();
    M[1].updatePID();
  }

  updateDeltas(tick_average - previous_tick_average, dir);
  previous_tick_average = tick_average;
}

// Command 2: Turn to IMU heading
void turn(double target_heading) {
  M[0].resetPosition();
  M[1].resetPosition();

  // Initialize desired change in heading and motor directions
  double diff;
  int dir[2];
  turnUpdate(target_heading, diff, dir);

  // Quickly turn to target heading until within 10 degrees
  static unsigned long cached_time = millis();
  while ((diff < 0) ? diff < -9 : diff > 9) {
    M[0].run(dir[0]);
    M[0].Setpoint = 40.0;
    M[1].run(dir[1]);
    M[1].Setpoint = 40.0;
    
    if (millis() - cached_time >= LOOPTIME) {
      cached_time = millis();
      M[0].updatePID();
      M[1].updatePID();
    }

    turnUpdate(target_heading, diff, dir);
  }
  stopWheels();

  turnUpdate(target_heading, diff, dir);

  // Slowly move to target heading until within 0.75 degreses
  while (abs(diff) > 0.75) {
    M[0].run(dir[0]);
    M[0].setSpeed(160);
    M[1].run(dir[1]);
    M[1].setSpeed(160);
    delay(80);
    stopWheels();

    turnUpdate(target_heading, diff, dir);
  }
}

// Command 2 helper function: Updates variables
void turnUpdate(double target_heading, double &diff, int (&dir)[2]) {
  bno.getEvent(&heading);
  diff = target_heading - heading.orientation.x;
  if (diff > 180) diff -= 360;
  else if (diff < -180) diff += 360;

  if (diff < 0) {
    dir[0] = FORWARD;
    dir[1] = BACKWARD;
  }
  else {
    dir[0] = BACKWARD;
    dir[1] = FORWARD;
  }
}

// Command 3 iterator: Drive straight by time
void driveTimeIter(int &command, unsigned long interval) {
  static unsigned long cached_time = 0;
  static unsigned long start_time = 0;
  static double previous_tick_average = 0.0;
  if (!initialized) {
    M[0].resetPosition();
    M[1].resetPosition();
    cached_time = millis();
    start_time = millis();
    previous_tick_average = 0.0;
    initialized = true;
  }

  if (millis() - cached_time >= LOOPTIME) {
    cached_time = millis();
    M[0].updatePID();
    M[1].updatePID();
  }

  if (millis() < start_time + interval) {
    M[0].run(FORWARD);
    M[0].Setpoint = 100.0;
    M[1].run(FORWARD);
    M[1].Setpoint = 100.0;
  }
  else {
    stopWheels();
    command = 0;
  }

  double tick_average = (abs(M[0].getPosition()) + abs(M[1].getPosition())) / 2.0;
  updateDeltas(tick_average - previous_tick_average, FORWARD);
  previous_tick_average = tick_average;
}

// Command 5 iterator: Wall follow with right side
void wallFollowRightIter(int &command, double distance) {
  // Back right is index 0, front right is index 1
  VL53L0X_RangingMeasurementData_t measures[2];

  tcaselect(0);
  IR[0].rangingTest(&measures[0], false);

  tcaselect(1);
  IR[1].rangingTest(&measures[1], false);

  double right_speed = 150.0, left_speed = 150.0;
  // Both IR sensors must be in range
  if (measures[0].RangeStatus != 4 && measures[1].RangeStatus != 4) {
    // Correct right
    if (measures[1].RangeMilliMeter < measures[0].RangeMilliMeter - 10) {
      if (measures[1].RangeMilliMeter < measures[0].RangeMilliMeter - 30) {
        right_speed = 200.0;
        left_speed = 100.0;
      }
      else {
        right_speed = 150.0;
        left_speed = 100.0;
      }
    }

    // Correct left
    else if (measures[1].RangeMilliMeter > measures[0].RangeMilliMeter + 10) {
      if (measures[1].RangeMilliMeter > measures[0].RangeMilliMeter + 30) {
        right_speed = 100.0;
        left_speed = 200.0;
      }
      else {
        right_speed = 100.0;
        left_speed = 150.0;
      }
    }

    // Drive the specified distance with correct speeds
    driveDistanceIter(command, distance, right_speed, left_speed);
  }

  // Just drive straight if we can't read the IRs
  else {
    driveDistanceIter(command, distance);
  }
}

// Command 6: Wall follow with left side
void wallFollowLeftIter(int &command, double distance) {
  // Back left is index 0, front left is index 1
  VL53L0X_RangingMeasurementData_t measures[2];

  tcaselect(3);
  IR[3].rangingTest(&measures[0], false);

  tcaselect(2);
  IR[2].rangingTest(&measures[1], false);

  double right_speed = 150.0, left_speed = 150.0;
  // Both IR sensors must be in range
  if (measures[0].RangeStatus != 4 && measures[1].RangeStatus != 4) {
    // Correct right
    if (measures[1].RangeMilliMeter < measures[0].RangeMilliMeter - 10) {
      if (measures[1].RangeMilliMeter < measures[0].RangeMilliMeter - 30) {
        right_speed = 100.0;
        left_speed = 200.0;
      }
      else {
        right_speed = 100.0;
        left_speed = 150.0;
      }
    }

    // Correct left
    else if (measures[1].RangeMilliMeter > measures[0].RangeMilliMeter + 10) {
      if (measures[1].RangeMilliMeter > measures[0].RangeMilliMeter + 30) {
        right_speed = 200.0;
        left_speed = 100.0;
      }
      else {
        right_speed = 150.0;
        left_speed = 100.0;
      }
    }

    // Drive the specified distance with correct speeds
    driveDistanceIter(command, distance, right_speed, left_speed);
  }

  // Just drive straight if we can't read the IRs
  else {
    driveDistanceIter(command, distance);
  }
}

// Command 7: Send the Raspberry Pi IMU heading and IR readings
void sendData() {
  // Send the IMU heading
  bno.getEvent(&heading);
  Serial.print(heading.orientation.x, 4); Serial.print("\n");

  // Send each IR reading
  VL53L0X_RangingMeasurementData_t measure;
  uint8_t t;
  for (t = 0; t < 8; t++) {
    tcaselect(t);
    IR[t].rangingTest(&measure, false);
    double distance;
    if (measure.RangeStatus != 4) {
      distance = (((double) measure.RangeMilliMeter) / 25.4) + 4.5;
    }
    else {
      distance = -1.0;
    }
    Serial.print(distance, 4);
    Serial.print("\n");
  }

  // Send the changes in x and y since the time data was sent
  Serial.print(x_delta, 4); Serial.print("\n");
  x_delta = 0.0;
  Serial.print(y_delta, 4); Serial.print("\n");
  y_delta = 0.0;
}

// Update changed distances with respect to x and y using changed encoder ticks and heading
void updateDeltas(double ticks_delta, uint8_t dir) {
  double distance = ticks_delta * INCHES_PER_TICK;
  double theta = (heading.orientation.x - 90) * PI / 180.0;

  if (dir == FORWARD) {
    x_delta += distance * cos(theta);
    y_delta += -1.0 * distance * sin(theta);
  }
  else { // BACKWARD
    x_delta -= distance * cos(theta);
    y_delta -= -1.0 * distance * sin(theta);
  }
}

// Stop the wheels as quickly as possible
void stopWheels() {
  M[0].run(STOP);
  M[0].setSpeed(0);
  M[1].run(STOP);
  M[1].setSpeed(0);

  long cached_M0_position = M[0].getPosition();
  long cached_M1_position = M[1].getPosition();
  unsigned long cached_time = millis();
  long count = 0;
  while (millis() - cached_time < 50) {
    if (cached_M0_position != M[0].getPosition() || cached_M1_position != M[1].getPosition()) {
      cached_M0_position = M[0].getPosition();
      cached_M1_position = M[1].getPosition();
      cached_time = millis();
    }
  }
}

// Initialize the I2C MUX and IMU (called once in setup)
void i2cSetup() {
  Serial.println("Beginning I2C setup . . .");
  // Identify each detected address at each port
  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;
      
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         Serial.print("TCA port "); Serial.print(t);
         Serial.print(" found address 0x");  Serial.print(addr,HEX);
         Serial.println(".");
      }
    }
  }

  // Initialize each IR sensor
  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    if (!IR[t].begin()) {
      Serial.print("Failed to boot IR[");
      Serial.print(t);
      Serial.println("].");
      while (true);
    }
  }

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  // Initialize the IMU
  if (!bno.begin()) {
    Serial.println("Failed to boot IMU.");
    while (true);
  }
  bno.setExtCrystalUse(true);
  delay(300);
  bno.getEvent(&heading);
  Serial.print("Initial heading is "); Serial.print(heading.orientation.x, 4);
  Serial.println(".");
}

// Open the specified port on the I2C MUX
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void raiseTheFlag() {
  uint8_t servonum = 3;
  int angle = 140;
  uint16_t pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, pulselen);
}
