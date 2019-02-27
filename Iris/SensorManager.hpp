#include <iostream>
#include <cstdint>
#include "./VL53L0X/VL53L0X.hpp"
#include "./Logger/logger.hpp"

#ifndef __SENSOR_MANAGER_INCLUDED__
#define __SENSOR_MANAGER_INCLUDED__
class SensorManager {
	private:
		int selector_A0;
		int selector_A1;
		int selector_A2;
		int currentSelector;
		VL53L0X** sensors;
		Logger* logger;
		
		void BaseSetup(VL53L0X*);
		void LongRangeSetup(VL53L0X*);
		void HighSpeedSetup(VL53L0X*);
		void HighAccuracySetup(VL53L0X*);
		void EmitSelector(int);
		
	public:
		SensorManager(std::string, std::string);
		void SetUp(uint8_t, int, int, int);
		bool Initialize();
		int SetSelector(int);
		int GetSelector();
		uint16_t ReadRange();
		uint16_t ReadRange(uint8_t);
};

#endif
