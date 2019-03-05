#include <iostream>
#include <cstdint>
#include "./VL53L0X/VL53L0X.hpp"
#include "./Logger/logger.hpp"
#include "./TCA9548A/TCAMux.h"

#ifndef __SENSOR_MANAGER_INCLUDED__
#define __SENSOR_MANAGER_INCLUDED__
class SensorManager {
	private:
		int size;
		int selector_A0;
		int selector_A1;
		int selector_A2;
		int currentSelector;
		VL53L0X** sensors;
		TCAMux* mux;
		Logger* logger;
		int* xshutPins;
		
		bool BaseSetup(VL53L0X*);
		void LongRangeSetup(VL53L0X*);
		void HighSpeedSetup(VL53L0X*);
		void HighAccuracySetup(VL53L0X*);
		void EmitSelector(int);
		
	public:
		SensorManager(std::string, std::string, int);
		void SetUp(int, int, int, int, int*);
		bool Initialize();
		void SetSelector(int);
		int GetSelector();
		uint16_t ReadRange();
		uint16_t ReadRange(uint8_t);
};

#endif
