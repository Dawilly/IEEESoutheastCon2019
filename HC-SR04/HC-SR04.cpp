#include "HC-SR04.h"

// Constructors

HCSR04::HCSR04() {
	TriggerPin = 53;
	EchoPin = 50;
	this->setPinMode();
}

HCSR04::HCSR04(int TriggerPin, int EchoPin) {
	this->TriggerPin = TriggerPin;
	this->EchoPin = EchoPin;
	this->RefreshRate = 500;
	this->setPinMode();
}

HCSR04::HCSR04(int TriggerPin, int EchoPin, ReadOperation ReadMode) {
	this->TriggerPin = TriggerPin;
	this->EchoPin = EchoPin;
	this->RefreshRate = 500;
	this->setReadMode(ReadMode);
	this->setPinMode();
}

HCSR04::HCSR04(int TriggerPin, int EchoPin, ReadOperation ReadMode, int RefreshRate) {
	this->TriggerPin = TriggerPin;
	this->EchoPin = EchoPin;
	this->RefreshRate = RefreshRate;
	this->setReadMode(ReadMode);
	this->setPinMode();
}

// Public Methods

double HCSR04::scan() {
	RawDistance = Trigger();
	distance = ConvertValue(this->ReadMode);
	return distance;
}

void HCSR04::setReadMode(ReadOperation ReadMode) {
	this->ReadMode = ReadMode;
	return;
}

long HCSR04::getLastScanRaw() {
	return this->RawDistance;
}

double HCSR04::getLastScan() {
	return this->distance;
}

double HCSR04::getLastScan(ReadOperation convertReadMode) {
	return this->ConvertValue(convertReadMode);
}

int HCSR04::getRefreshRate() {
	return this->RefreshRate;
}

// Protected Methods

void HCSR04::setPinMode() {
	pinMode(TriggerPin, OUTPUT);
	pinMode(EchoPin, INPUT);
	return;
}

long HCSR04::Trigger() {
	long reading;
	digitalWrite(this->TriggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(this->TriggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(this->TriggerPin, LOW);
	reading = pulseIn(EchoPin, HIGH);
	return reading;
}

double HCSR04::ConvertValue(ReadOperation AsReadMode) {
	double results;

	switch (AsReadMode) {
		case inches:
			results = RawDistance / 74.0 / 2.0;
			break;
		case centimeters:
			results = RawDistance / 29.0 / 2.0;
			break;
		default:
			results = -1;
			break;
	}

	return results;
}