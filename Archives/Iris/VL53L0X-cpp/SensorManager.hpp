#include <iostream>
#include <cstdint>
#include "TofSensor.hpp"
#include "./VL53L0X/VL53L0X.hpp"
#include "./Logger/logger.hpp"
#include "./TCA9548A/TCAMux.h"

#ifndef __SENSOR_MANAGER_INCLUDED__
#define __SENSOR_MANAGER_INCLUDED__
class SensorManager {
	private:
		int size;
		int currentSelector;
		TofSensor** sensors;
		TCAMux* mux;
		Logger* logger;
		Logger* errorLog;
		
	public:
		SensorManager(std::string, std::string, int);
		void SetUp(int);
		void Initialize();
		void SetSelector(int);
		int GetSelector();
		uint16_t ReadRange();
		uint16_t ReadRange(int);
};

#endif
