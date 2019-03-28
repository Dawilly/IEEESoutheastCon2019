#include <iostream>
#include <wiringPi.h>
#include "SensorManager.hpp"

using namespace std;

SensorManager::SensorManager(string filename, string limiter, int size) {
	this->size = size;
	sensors = new TofSensor*[size];
	mux = new TCAMux;
	logger = new Logger(filename, limiter);
	errorLog = new Logger("Error.log", "\n");
}

void SensorManager::SetUp(int value) {
	int i;
	int bit;

	mux->Initialize();
	
	for (i = 0; i < size; i++) {
		bit = value % 2;
		value = value / 2;
		if (bit) {
			sensors[i] = new TofSensor(i);
		} else {
			sensors[i] = nullptr;
		}
	}
	SetSelector(0);
	return;
}

void SensorManager::Initialize() {
	int i;
	for (i = 0; i < size; i++) {
		if (sensors[i] != nullptr) {
			sensors[i]->Initialize();
		}
	}

	return;
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
	delayMicroseconds(100);
	value = sensors[currentSelector]->readRange();
	delayMicroseconds(100);
	return value;
}

uint16_t SensorManager::ReadRange(int value) {
	SetSelector(value);
	return ReadRange();
}
