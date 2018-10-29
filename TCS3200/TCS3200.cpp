#include "TCS3200.h"

// Constructors

TCS3200::TCS3200() {

}

TCS3200::TCS3200(int* sPinArray, int output) {
	this->OutputPin = output;
	for (int i = 0; i < 4; i++) {
		SPins[i] = sPinArray[i];
	}
}

TCS3200::TCS3200(int s0pin, int s1pin, int s2pin, int s3pin, int out) {
	this->OutputPin = out;
	this->SPins[0] = s0pin;
	this->SPins[1] = s1pin;
	this->SPins[2] = s2pin;
	this->SPins[3] = s3pin;
}

// Public

void TCS3200::scan() {
	scanRed();
	delay(50);
	scanGreen();
	delay(50);
	scanBlue();
	delay(50);
}

void TCS3200::printResults() {
	Serial.print("Red = ");
	Serial.print(this->redValue);
	Serial.print(" Green = ");
	Serial.print(this->greenValue);
	Serial.print(" Blue = ");
	Serial.print(this->blueValue);
	Serial.print("\n");
}

/*
String TCS3200::toString() {
	String results;
	results = "Red = " + this->redValue;
	results += "\tGreen = " + this->greenValue;
	results += "\tBlue" + this->blueValue;
	results += "\n";
	return results;
}
*/

// Protected

void TCS3200::setPins() {
	for (int i = 0; i < 4; i++) {
		pinMode(this->SPins[i], OUTPUT);
	}
	pinMode(OutputPin, INPUT);
	digitalWrite(SPins[0], HIGH);
	digitalWrite(SPins[1], HIGH);
}

void TCS3200::scanRed() {
	scanColor(red, &redValue);
}

void TCS3200::scanGreen() {
	scanColor(green, &greenValue);
}

void TCS3200::scanBlue() {
	scanColor(blue, &blueValue);
}

// Private 

void TCS3200::scanColor(pdType type, int* value) {
	digitalWrite(SPins[2], pinSettings[type].s2pin);
	digitalWrite(SPins[3], pinSettings[type].s3pin);
	(*value) = pulseIn(OutputPin, LOW);
}
