/*
 Name:		SDS01A.ino
 Created:	11/19/2018 4:44:09 PM
 Author:	Weil
*/

#include "SDS01A.h"

SDS01A sensor;
double distance;

// the setup function runs once when you press reset or power the board
void setup() {
	sensor = SDS01A(A0, raw);
	Serial.begin(9600);
	Serial.println("Testing SDS01A Sensor:");
}

// the loop function runs over and over again until power down or reset
void loop() {
	distance = sensor.scan();
	Serial.print("Reading: ");
	Serial.print(distance);
  Serial.print("\n");
	delay(150);
}
