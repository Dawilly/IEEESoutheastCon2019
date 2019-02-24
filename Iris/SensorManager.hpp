#include <iostream>
#include <cstdint>
#include "./VL53L0X/VL53L0X.hpp"
#include "./Logger/logger.hpp"

#ifndef __SENSOR_MANAGER_INCLUDED__
#define __SENSOR_MANAGER_INCLUDED__
class SensorManager {
	private:
		VL53L0X* sensors;
		Logger* logger;
		
	public:
		SensorManager(std::string, std::string);
		void SetUp(uint8_t);
		bool Initialize();
}

#endif
