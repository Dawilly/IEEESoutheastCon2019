#include <iostream>
#include <cstdint>
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

#ifndef __VL53__INCLUDED__
#define __VL53__INCLUDED__

class VL53L0X {
	private:
		int input;
		VL53L0X_Dev_t* unit;
		VL53L0X_Error status;
		VL53L0X_RangingMeasurementData_t *data;

	public:
		VL53L0X(int value);
		void Initialize();
		VL53L0X_Dev_t* getUnit();
		uint16_t readRange();
};

#endif
