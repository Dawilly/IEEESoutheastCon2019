#include "TCS3200.h"

// Constructors

TCS3200::TCS3200() {

}

/// <summary>
/// Constructor for the TCS3200 class to interact with an TCS3200 color sensor. 
/// </summary>
TCS3200::TCS3200(int* sPinArray, int output) {
	this->OutputPin = output;
	for (int i = 0; i < 4; i++) {
		SPins[i] = sPinArray[i];
	}
	this->enableLED = false;
	initalize();
}

TCS3200::TCS3200(int s0pin, int s1pin, int s2pin, int s3pin, int out) {
	this->OutputPin = out;
	this->SPins[0] = s0pin;
	this->SPins[1] = s1pin;
	this->SPins[2] = s2pin;
	this->SPins[3] = s3pin;
	this->enableLED = false;
	initalize();
}

TCS3200::TCS3200(int s0pin, int s1pin, int s2pin, int s3pin, int out, int led) {
	this->OutputPin = out;
	this->SPins[0] = s0pin;
	this->SPins[1] = s1pin;
	this->SPins[2] = s2pin;
	this->SPins[3] = s3pin;
	this->LEDPin = led;
	this->enableLED = true;
	initalize();
}

// Public

void TCS3200::scan() {

}

void TCS3200::printResults(int mode = 0) {
	Serial.print("Red = ");
	Serial.print(colorData.red);
	Serial.print(" Green = ");
	Serial.print(colorData.green);
	Serial.print(" Blue = ");
	Serial.print(colorData.blue);
	if (mode > 1) {
		Serial.print("(Raw : Red = ");
		Serial.print(rawData.red);
		Serial.print(" Green = ");
		Serial.print(rawData.green);
		Serial.print(" Blue = ");
		Serial.print(rawData.blue);
		Serial.print(" )");
	}
	Serial.print("\n");
	return;
}

void TCS3200::calibration() {

}

// Protected

void TCS3200::initalize() {
	setPins();
	clearData(rawData);
	clearData(colorData);
}

void TCS3200::setPins() {
	for (int i = 0; i < 4; i++) {
		pinMode(this->SPins[i], OUTPUT);
	}
	pinMode(OutputPin, INPUT);
	if (enableLED) {
		pinMode(LEDPin, OUTPUT);
		digitalWrite(LEDPin, HIGH);
	}
	digitalWrite(SPins[0], HIGH);
	digitalWrite(SPins[1], HIGH);
}

template <typename T>
void TCS3200::clearData(Data<T> data) {
	data.red = 0;
	data.green = 0;
	data.blue = 0;
	data.clear = 0;
}

void TCS3200::scanRaw() {
	scanColor(red, &rawData.red);
	scanColor(green, &rawData.green);
	scanColor(blue, &rawData.blue);
}

void TCS3200::scanRaw(Data<float> data) {
	scanColor(red, &data.red);
	scanColor(green, &data.green);
	scanColor(blue, &data.blue);
}

// Private 

void TCS3200::scanColor(pdType type, float *value) {
	float color = 0;
	float fvalue = 0;
	float results = 0;
	int n = 0;

	digitalWrite(SPins[2], pinSettings[type].s2pin);
	digitalWrite(SPins[3], pinSettings[type].s3pin);

	for (int i = 0; i < sampleSize; i++) {
		color = pulseIn(OutputPin, digitalRead(OutputPin) == HIGH ? LOW : HIGH);
		if (color / sampleSize > fvalue / 1.5) {
			fvalue = (fvalue + color) / sampleSize;
			results += color;
			n++;
		}
	}
	(*value) = results / n;
	return;
}

void TCS3200::calibrateBlack() {
	Data<float> darkcal;
	clearData(darkcal);
	scanRaw(darkcal);
	_darkcal = darkcal;
}

void TCS3200::calibrateWhite() {

}