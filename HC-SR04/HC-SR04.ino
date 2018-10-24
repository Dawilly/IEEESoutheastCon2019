#include "Arduino.h"
#include "HC-SR04.h"
/*
 Name:		HC_SR04.ino
 Created:	10/18/2018 11:00:56 PM
 Author:	Weil
*/

HCSR04 sensor;
double distance;

// the setup function runs once when you press reset or power the board
void setup() {
	sensor = HCSR04(50, 53, inches, 250);
	Serial.begin(9600);
	Serial.println("Testing HC-SR04 Sensor:");
}

// the loop function runs over and over again until power down or reset
void loop() {
	distance = sensor.scan();
	Serial.print("Reading: ");
	Serial.print(sensor.getLastScanRaw());
	Serial.print(" raw - ");
	Serial.print(distance);
	Serial.print(" inches\n");
	delay(sensor.getRefreshRate());
}
