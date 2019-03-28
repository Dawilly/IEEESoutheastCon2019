#include <iostream>
#include "./VL53L0X/VL53L0X.hpp"
#include "TofSensor.hpp"

using namespace std;

TofSensor::TofSensor(int number) {
	this->number = number;
	this->settings = Normal;
	this->tof = new VL53L0X;
}

TofSensor::TofSensor(int number, VL53Settings settings) {
	this->number = number;
	this->settings = settings;
	this->tof = new VL53L0X;
}

void TofSensor::Initialize() {
	BaseSetup();
	if (settings == Normal) return;
	
	switch (settings) {
		case LongRange:
		case LongRangeHighSpeed:
		case LongRangeHighAccuracy:
			LongRangeSetup();
			break;
	}
	
	switch (settings) {
		case HighSpeed:
		case LongRangeHighSpeed:
			HighSpeedSetup();
			break;
		case HighAccuracy:
		case LongRangeHighAccuracy:
			HighAccuracySetup();
			break;
	}
	
	return;
}

uint16_t TofSensor::readRange() {
	try {
		distance = tof->readRangeSingleMillimeters();
	} catch (const exception &error) {
		//Report error;
		distance = 8096;
	}
	
	if (tof->timeoutOccurred()) {
		//Report timeout;
	}
	
	return distance;
}

void TofSensor::BaseSetup() {
	try {
		tof->Initialize();
		tof->setTimeout(200);
		tof->setMeasurementTimingBudget(20000);
	} catch (const exception &error) {
		//Report error
	}
}

void TofSensor::LongRangeSetup() {
	try {
		// Lower the return signal rate limit
		tof->setSignalRateLimit(0.1);
		// Increase laser pulse periods
		tof->setVcselPulsePeriod(VcselPeriodPreRange, 18);
		tof->setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	} catch (const exception &error) {
		//Report Error
	}	
}

void TofSensor::HighSpeedSetup() {
	try {
		// Reduce timing budget to 20 ms (instead of the default 33 ms)
		tof->setMeasurementTimingBudget(20000);
	} catch (const exception &error) {
		//Report Error
	}
}

void TofSensor::HighAccuracySetup() {
	try {
		// Increase timing budget to 200 ms (instead of the default 33 ms)
		unit->setMeasurementTimingBudget(200000);
	} catch (const exception &error) {
		//Report Error
	}	
}
	