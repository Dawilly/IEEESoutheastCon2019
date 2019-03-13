#include <iostream>
#include <cstdint>
#include "VL53L0X.hpp"
#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

using namespace std;

VL53L0X::VL53L0X(int value) {
	input = value;
	unit = new VL53L0X_Dev_t;
	data = new VL53L0X_RangingMeasurementData_t;
	unit->I2cDevAddr = 0x29;
	this->status = VL53L0X_ERROR_NONE;
	char str[] = {"/dev/i2c-1"};
	unit->fd = VL53L0X_i2c_init(str, unit->I2cDevAddr);
}

void VL53L0X::Initialize() {
	if (unit->fd < 0) {
		status = VL53L0X_ERROR_CONTROL_INTERFACE;
		cout << "Failed to initialize VL53L0X on input: " << input;
	}

	status = VL53L0X_DataInit(unit);

	return;
}

VL53L0X_Dev_t* VL53L0X::getUnit() {
	return unit;
}

uint16_t VL53L0X::readRange() {
	status = VL53L0X_PerformSingleRangingMeasurement(unit, data);
	if (status == VL53L0X_ERROR_NONE) {
		return data->RangeMilliMeter;
	} else {
		return 0;
	}
}
