#include <iostream>
#include "SensorManager.hpp"

using namespace std;

SensorManager::SensorManager(string filename, string limiter) {
	sensors = new VL53L0X[8];
	logger = new Logger(filename, limiter);

}

void SensorManager::SetUp(uint8_t value) {
	int i;
	for (i = 0; i < 8; i++) {
		value >>> 1
	}
}

bool SensorManager::Initialize() {
	
}
