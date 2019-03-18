

#ifndef __TOFSENSOR__INCLUDED__
#define __TOFSENSOR__INCLUDED__

#include <iostream>
#include <cstdint>
#include "./VL53L0X/VL53L0X.hpp"

enum VL53Settings {
	Normal,
	LongRange,
	HighSpeed,
	HighAccuracy,
	LongRangeHighSpeed,
	LongRangeHighAccuracy
}

class TofSensor {
	private:
		int number;
		VL53Settings settings;
		VL53L0X* tof;
		uint16_t distance;
		
		void BaseSetup();
		void LongRangeSetup();
		void HighSpeedSetup();
		void HighAccuracySetup();
	public:
		TofSensor(int);
		TofSensor(int, VL53Settings);
		~TofSensor();
		void Initialize();
		uint16_t readRange();
};

#endif
