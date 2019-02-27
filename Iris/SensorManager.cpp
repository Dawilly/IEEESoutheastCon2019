#include <iostream>
#include <wiringPi.h>
#include "SensorManager.hpp"

using namespace std;

SensorManager::SensorManager(string filename, string limiter) {
	sensors = new VL53L0X*[8];
	logger = new Logger(filename, limiter);
}

void SensorManager::SetUp(uint8_t value, int A0, int A1, int A2) {
	int i;
	int bit;
	
	selector_A0 = A0;
	selector_A1 = A1;
	selector_A2 = A2;
	
	for (i = 0; i < 8; i++) {
		bit = value % 2;
		value = value / 2;
		if (bit) {
			sensors[i] = new VL53L0X;
		} else {
			sensors[i] = nullptr;
		}
	}
	SetSelector(0);
}

bool SensorManager::Initialize() {
	pinMode(selector_A0, OUTPUT);
	pinMode(selector_A1, OUTPUT);
	pinMode(selector_A2, OUTPUT);

	int i;
	for (i = 0; i < 8; i++) {
		if (sensors[i] != nullptr) {
			BaseSetup(sensors[i]);
		}
	}
	return true;
}

void SensorManager::BaseSetup(VL53L0X* unit) {
	try {
		unit->initialize();
		unit->setTimeout(200);
	} catch (const exception &error) {
		//Log Error
		exit(-1);
	}
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
}

void SensorManager::SetSelector(int value) {
	currentSelector = value;
	EmitSelector(currentSelector);
	delayMicroseconds(100);
}

int SensorManager::GetSelector() {
	return currentSelector;
}

uint16_t SensorManager::ReadRange() {
	if (sensors[i] == nullptr) return 0;
	return sensors[i]->readRangeSingleMillimeters();
}

uint16_t SensorManager::ReadRange(uint8_t setPin) {
	currentSelector = setPin;
	EmitSelector(currentSelector);
	return this.ReadRange();
}
