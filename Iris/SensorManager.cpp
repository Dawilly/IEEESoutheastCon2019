#include <iostream>
#include <wiringPi.h>
#include "SensorManager.hpp"

using namespace std;

SensorManager::SensorManager(string filename, string limiter, int size) {
	this->size = size;
	sensors = new VL53L0X*[size];
	mux = new TCAMux;
	logger = new Logger(filename, limiter);
	xshutPins = new int[size];
}

void SensorManager::SetUp(int value, int A0, int A1, int A2, int* xPins) {
	int i;
	int bit;

	mux->Initialize();
	
	selector_A0 = A0;
	selector_A1 = A1;
	selector_A2 = A2;
	
	for (i = 0; i < size; i++) {
		bit = value % 2;
		value = value / 2;
		if (bit) {
			xshutPins[i] = xPins[i];
			sensors[i] = new VL53L0X(xPins[i]);
			sensors[i]->powerOff();
		} else {
			sensors[i] = nullptr;
		}
	}
	SetSelector(0);
}

bool SensorManager::Initialize() {
	bool status;
	pinMode(selector_A0, OUTPUT);
	pinMode(selector_A1, OUTPUT);
	pinMode(selector_A2, OUTPUT);

	int i;
	for (i = 0; i < size; i++) {
		if (sensors[i] != nullptr) {
			sensors[i]->powerOn();
			status = BaseSetup(sensors[i]);
			sensors[i]->powerOff();
			if (!status) return false;
		}
	}

	return true;
}

bool SensorManager::BaseSetup(VL53L0X* unit) {
	try {
		unit->initialize();
		unit->setTimeout(200);
		unit->setMeasurementTimingBudget(20000);
	} catch (const exception &error) {
		cerr << "Error initializing sensor with reason:" << endl << error.what() << endl;
		//Log Error
		return false;
	}
	
	return true;
}

void SensorManager::LongRangeSetup(VL53L0X* unit) {
	try {
		unit->setSignalRateLimit(0.1);
		unit->setVcselPulsePeriod(VcselPeriodPreRange, 18);
		unit->setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	} catch (const exception &error) {
		//Log Error
		exit(-2);
	}
}

void SensorManager::HighSpeedSetup(VL53L0X* unit) {
	try {
		unit->setMeasurementTimingBudget(20000);
	} catch (const exception &error) {
		//Log Error
		exit(-3);
	}
}

void SensorManager::HighAccuracySetup(VL53L0X* unit) {
	try {
		unit->setMeasurementTimingBudget(200000);
	} catch (const exception &error) {
		//Log Error
		exit(-4);
	}
}

void SensorManager::EmitSelector(int value) {
	int vA0 = value % 2;
	int vA1 = (value / 2) % 2;
	int vA2 = ((value / 2) / 2) % 2;
	digitalWrite(selector_A0, vA0);
	digitalWrite(selector_A1, vA1);
	digitalWrite(selector_A2, vA2);
	return;
}

void SensorManager::SetSelector(int value) {
	currentSelector = value;
	EmitSelector(currentSelector);
	delayMicroseconds(1000);
}

int SensorManager::GetSelector() {
	return currentSelector;
}

uint16_t SensorManager::ReadRange() {
	uint16_t value;
	if (sensors[currentSelector] == nullptr) return 0;
	sensors[currentSelector]->powerOn();
	delayMicroseconds(100);
	value = sensors[currentSelector]->readRangeSingleMillimeters();
	sensors[currentSelector]->powerOff();
	delayMicroseconds(100);
	return value;
}

uint16_t SensorManager::ReadRange(uint8_t setPin) {
	currentSelector = setPin;
	EmitSelector(currentSelector);
	return ReadRange();
}
