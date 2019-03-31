#include "IRSensor.h"

using namespace std;


/// Constructor ///

/// IRSensor()
///
/// Description: Creates an instance representing a VL53L0X
/// 			 distance sensor.
IRSensor::IRSensor() {
	this->success = false;
	this->SetUp(false, 0);
}

/// IRSensor(bool debug)
///
/// Description: Creates an instance representing a VL53L0X
///				 distance sensor.
/// Parameter:
/// @bool debug - Enables debug mode.
IRSensor::IRSensor(bool debug) {
	this->success = false;
	this->SetUp(debug, 0);
}

/// IRSensor(int offset, bool debug)
///
/// Description: Creates an instance representing a VL53L0X
///				 distance sensor.
/// Parameters:
/// @int offset - The value to offset the measurements by
///					   in millimeters.
/// @bool debug - Enables debug mode.
IRSensor::IRSensor(int offset, bool debug) {
	this->success = false;
	this->SetUp(debug, offset);
}

/// Destructor ///

/// ~IRSensor()
///
/// Description: Deallocates an IRSensor instance from memory.
IRSensor::~IRSensor() {
	delete this->sensor;
	delete this->measurement;
}

/// Methods ///

/// SetUp(bool debug, int offset)
///
/// Description: Initalizes the required objects and values.
///	Parameters:
/// @bool debug - Enables debug mode.
/// @int offset - The value to offset the measurements by
///					   in millimeters.
void IRSensor::SetUp(bool debug, int offset) {
	if (this->success) return;
	
	this->sensor = new Adafruit_VL53L0X();
	this->measurement = new VL53L0X_RangingMeasurementData_t();
	this->debug = debug;
	this->success = true;
	this->offset = offset;
	
	if (!sensor->begin()) {
		delete sensor;
		delete measurement;
		this->success = false;
	}	
}

/// readRange()
///
/// Description: Calls the IRSensor to read the current distance
///				 from the closet object.
/// Returns: The distance in millimeters, as an uint16_t.
int IRSensor::readRange() {
	sensor->rangingTest(measurement, debug);
	return ((int)measurement->RangeMilliMeter - offset);
}

/// lastReading()
///
/// Description: Gets the last reading from readRange as a uint16_t. 
///				 Does not use the sensor to read a new distance value.
/// Returns: The distance from the latest reading, in millimeters as an uint16_t.
int IRSensor::lastReading() {
	return ((int)measurement->RangeMilliMeter - offset);
}

/// RangeStatus()
///
/// Description: Gets the Status of the VL53L0X sensor.
/// Returns: An integer value representing the the status of the
///			 VL53L0X sensor. Refer to the VL53L0X for descriptions.
int IRSensor::RangeStatus() {
	return measurement->RangeStatus;
}

/// adjustOffset(uint16_t)
///
/// Description: Set the offset value for the measurements of the IRSensor.
void IRSensor::adjustOffset(int offset) {
	this->offset = offset;
}