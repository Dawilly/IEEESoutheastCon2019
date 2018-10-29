#include "Arduino.h"
#include "TCS3200.h"
/*
 Name:		TCS3200.ino
 Created:	10/24/2018 3:18:21 PM
 Author:	Weil
*/

TCS3200 colorSenor;

// the setup function runs once when you press reset or power the board
void setup() {
	colorSenor = TCS3200(47, 45, 43, 41, 39);
	Serial.begin(9600);
	Serial.println("Testing TCS3200 Color Sensor:");
}

// the loop function runs over and over again until power down or reset
void loop() {
	delay(250);
	colorSenor.scan();
	delay(250);
	colorSenor.printResults();
}
