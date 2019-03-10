#include <iostream>
#include <wiringPi.h>
#include "SensorManager.hpp"

using namespace std;

SensorManager::SensorManager(string filename, string limiter, int size) {
	this->size = size;
	sensors = new VL53L0X*[size];
	mux = new TCAMux;
	//logger = new Logger(filename, limiter);
}

void SensorManager::SetUp(int value) {
	int i;
	int bit;

	mux->Initialize();
	
	for (i = 0; i < size; i++) {
		bit = value % 2;
		value = value / 2;
		if (bit) {
			sensors[i] = new VL53L0X(i);
		} else {
			sensors[i] = nullptr;
		}
	}
	SetSelector(0);
}

void SensorManager::Initialize() {
	int i;
	for (i = 0; i < size; i++) {
		if (sensors[i] != nullptr) {
			BaseSetup(sensors[i]);
		}
	}

	return;
}

void SensorManager::BaseSetup(VL53L0X* unit) {
	return unit->Initialize();
}

void SensorManager::LongRangeSetup(VL53L0X* unit) {

}

void SensorManager::HighSpeedSetup(VL53L0X* unit) {

}

void SensorManager::HighAccuracySetup(VL53L0X* unit) {

}

void SensorManager::SetSelector(int value) {
	currentSelector = value;
	mux->Switch(value);
}

int SensorManager::GetSelector() {
	return currentSelector;
}

uint16_t SensorManager::ReadRange() {
	uint16_t value;
	if (sensors[currentSelector] == nullptr) return 0;
	//delayMicroseconds(100);
	value = sensors[currentSelector]->readRange();
	//delayMicroseconds(100);
	return value;
}

uint16_t SensorManager::ReadRange(int value) {
	SetSelector(value);
	return ReadRange();
}
