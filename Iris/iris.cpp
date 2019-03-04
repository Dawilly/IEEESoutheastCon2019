#include "VL53L0X/VL53L0X.hpp"
#include "Logger/logger.hpp"
#include "SensorManager.hpp"
#include <wiringPi.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
	int i;
	int A[3] = { 27, 28, 29 };
	int xShut[8] = { 22, 23, 0, 0, 0, 0, 0, 0 };

	
	//0000 0001
	int sp = 0x01;
	
	wiringPiSetup();
	
	SensorManager eyes("a", "a", 8);
	
	eyes.SetUp(sp, A[0], A[1], A[2], xShut);
	eyes.Initialize();
	eyes.SetSelector(0);
	
	while(1) {
		for (i = 0; i < 8; i++) {
			eyes.SetSelector(i);
			cout << "Sensor #" << i << ": " << eyes.ReadRange() << " mm " << endl;
		}
	}
	
	return 0;
}

