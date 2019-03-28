#include "WallFollow/IRSensor.h"

using namespace std;
		
IRSensor::IRSensor(bool debug = false) {
	this->sensor = new Adafruit_VL53L0X();
	this->measurement = new VL53L0X_RangingMeasurementData_t();
	this->debug = debug;
	this->success = true;
	
	if (!sensor->begin()) {
		delete sensor;
		delete measurement;
		this->success = false;
	}
}

uint16_t IRSensor::readRange() {
	sensor->rangingTest(measurement, debug);
	return measurement->RangeMilliMeter;
}

uint16_t IRSensor::lastReading() {
	return measurement->RangeMilliMeter;
}

int IRSensor::RangeStatus() {
	return measurement->RangeStatus;
}
