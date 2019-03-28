#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

VL53L0X_Error VL53L0X_setup(VL53L0X_Dev_t*);
void print_pal_error(VL53L0X_Error);
void print_range_status(VL53L0X_RangingMeasurementData_t *);