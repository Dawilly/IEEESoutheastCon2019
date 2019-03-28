#include <Adafruit_VL53L0X.h>
#include <Wire.h>

#ifndef __IRSENSOR__INCLUDED__
#define __IRSENSOR__INCLUDED__

class IRSensor {
	
	private:
		Adafruit_VL53L0X* sensor;
		VL53L0X_RangingMeasurementData_t* measurement;
		bool debug;
		
	public:
		bool success;
		
		IRSensor(bool);
		//~IRSensor();
		uint16_t readRange();
		uint16_t lastReading();
		int RangeStatus();
};

#endif
