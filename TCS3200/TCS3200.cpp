#include "TCS3200.h"

#define DEBUG true

// Constructors

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
	scanRaw();
	convertRaw2RGB(rawData);
}

void TCS3200::printResults(int mode = 0) {
	Serial.print("Red = ");
	Serial.print(colorData.red);
	Serial.print(" Green = ");
	Serial.print(colorData.green);
	Serial.print(" Blue = ");
	Serial.print(colorData.blue);
	if (mode > 0) {
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
	#if DEBUG
	bool waitDark = true;
	Serial.println("Set up calibration for black. Press enter when ready.");
	while (waitDark) {
		if (Serial.available() > 0) {
			 waitDark = false;
		}
		while(Serial.available() > 0) {
			Serial.readString();
		}
		Serial.flush();
	}
	#endif
	calibrateBlack();
	
	#if DEBUG
	bool waitLight = true;
	Serial.println("Set up calibration for white. Press enter when ready.");
	while (waitLight) {
		if (Serial.available() > 0) {
			waitLight = false;
		}
		while(Serial.available() > 0) {
			Serial.readString();
		}
		Serial.flush();
	}
	#endif
	calibrateWhite();
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

void TCS3200::convertRaw2RGB(Data<float> data) {
	colorData.red = calculateRGB(data.red, _darkcal.red, _whitecal.red);
	colorData.green = calculateRGB(data.green, _darkcal.green, _whitecal.green);
	colorData.blue = calculateRGB(data.blue, _darkcal.blue, _whitecal.blue);
}

// Private 

int TCS3200::calculateRGB(float rawVal, float darkVal, float whiteVal) {
	int x;
	x = (rawVal - darkVal) * 255;
	x /= (whiteVal - darkVal);
	if (x < 0) {
		return 0;
	} else if (x > 255) {
		return 255;
	} else {
		return x;
	}
}

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
	printRaw(darkcal);
	_darkcal = darkcal;
}

void TCS3200::calibrateWhite() {
	Data<float> whitecal;
	clearData(whitecal);
	scanRaw(whitecal);
	printRaw(whitecal);
	_whitecal = whitecal;
}

void TCS3200::printRaw(Data<float> raw) {
	Serial.print("Red = ");
	Serial.print(raw.red);
	Serial.print(" Green = ");
	Serial.print(raw.green);
	Serial.print(" Blue = ");
	Serial.print(raw.blue);
	Serial.print("\n");
}