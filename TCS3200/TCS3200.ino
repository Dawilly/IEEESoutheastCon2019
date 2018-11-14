#include "Arduino.h"
#include "TCS3200.h"
/*
 Name:		TCS3200.ino
 Created:	10/24/2018 3:18:21 PM
 Author:	Weil
*/

TCS3200 colorSensor = TCS3200(47, 45, 43, 41, 39, 33);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	Serial.println("Testing TCS3200 Color Sensor:");
	colorSensor.calibration();
}

// the loop function runs over and over again until power down or reset
void loop() {
	delay(250);
	colorSensor.scan();
	delay(250);
	colorSensor.printResults(1);
}
