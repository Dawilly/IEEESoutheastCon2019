#include "WallFollow.h"

using namespace std;

/// Constructor ///

/// WallFollow(IRSensor** sensors, bool debug = false)
///
/// Description: Creates an instance of the Algorithm: WallFollow
/// Parameters:
/// @IRSensor** sensors - Pointers to the IRSensors instances.
/// @bool debug - Enables debug mode. False by default.
WallFollow::WallFollow(IRSensor** sensors, bool debug = false) {
	this->sensors = sensors;
	this->selectedSensor_0 = -1;
	this->selectedSensor_1 = -1;
	this->debug = debug;
}

/// Public Methods ///

/// Act()
///
/// Description: Performs the operations for the wall follow algorithm.
void WallFollow::Act() {
	bool needsAdjusting = checkSensors(selectedSensor_0, selectedSensor_1, threshold);
	
	if (needsAdjusting) {
		/// Call Drive to Turn.
		sprintf(buf, "Adjustment is needed!\n");
		Serial.print(buf);
	}
	
	return;
}

/// Act(int s0, int s1)
///
/// Description: Performs the operations for the wall follow algorithm, after
///				 setting the sensors to follow.
///	Parameters:
/// @int s0 - The I2C Mux pin for the first sensor.
/// @int s1 - The I2C Mux pin for the second sensor.
void WallFollow::Act(int s0, int s1) {
	SetSensorsToTrace(s0, s1);
	Act();
}

/// Initialize(int s0, int s1, uint16_t threshold)
///
/// Description: Sets the pins of the corresponding sensors that are facing
///				 the wall along with the difference threshold between the two
///				 sensors.
///	Parameters:
/// @int s0 - The I2C Mux pin for the first sensor.
/// @int s1 - The I2C Mux pin for the second sensor.
/// @uint16_t threshold - The maximum difference allowed.
void WallFollow::Initialize(int s0, int s1, uint16_t threshold) {
	selectedSensor_0 = s0;
	selectedSensor_1 = s1;
	this->threshold = threshold;
}

/// SetSensorsToTrace(int s0, int s1)
///
/// Description: Sets the pins of the corresponding sensors that are facing
///				 the wall.
/// Parameters:
/// @int s0 - The I2C Mux pin for the first sensor.
/// @int s1 - The I2C Mux pin for the second sensor.
void WallFollow::SetSensorsToTrace(int s0, int s1) {
	selectedSensor_0 = s0;
	selectedSensor_1 = s1;
	return;
}

///////////////////////
/// Private Methods ///
///////////////////////

/// tcaselect(uint8_t pin)
///
/// Description: Sets the active input pin on the multiplexer. 
/// Parameter:
/// @uint8_t pin - The input pin(s) of the multiplexer to set. Valid values range
///                from 0 to 7. Values above 7 and under 0 are ignored. 
void WallFollow::TCASELECT(uint8_t pin) {
	if (pin > 7  || pin < 0) return;
	Wire.beginTransmission(TCAADDR);
	Wire.write(1 << pin);
	Wire.endTransmission();
}

/// checkSensors(int s1, int s2, uint16_t threshold)
///
/// Description: Takes one measurement from two different sensors, each.
///              Calculates the difference and compares with threshold.
/// Parameters:
/// @int s1 - The first sensor to take a measurement from.
/// @int s2 - The second sensor to take a measurement from.
/// @uint16_t threshold - The maximum difference allowed between the two
///                       distance readings.
/// Return:
/// True - If the measurement between s1 and s2 is less than or equal to 
///        the threshold.
/// False - Otherwise.
bool WallFollow::checkSensors(int s0, int s1, uint16_t limit) {
	uint16_t val0 = 0;
	uint16_t val1 = 0;
	TCASELECT(selectedSensor_0);
	delay(10);
	val0 = sensors[selectedSensor_0]->readRange();
	
	TCASELECT(selectedSensor_1);
	delay(10);
	val1 = sensors[selectedSensor_1]->readRange();
	
	if (debug) {
		printSensorData(selectedSensor_0);
		printSensorData(selectedSensor_1);
	}
	
	return (abs(val0 - val1) >= threshold);
}

/// printSensorData(int pin)
///
/// Description: Prints out the data the sensors measured.
/// Parameters:
/// @int pin - The pin number containing the sensor we want to read.
void WallFollow::printSensorData(int pin) {
	if (sensors[pin] != NULL && sensors[pin]->RangeStatus() != 4) {
		sprintf(buf, "Distance (mm): %d\n", sensors[pin]->lastReading());
		Serial.print(buf);
	} else if (sensors[pin] == NULL) {
		sprintf(buf, "Pin %d does not contain a sensor/failed to boot\n", pin);
		Serial.print(buf);
	} else {
		sprintf(buf, "Sensor %d is out of range\n", pin);
		Serial.print(buf);
	}
	return;	
}
