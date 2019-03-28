#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "IRSensor.h"

#ifndef __WALLFOLLOW_ALGORITHM__
#define __WALLFOLLOW_ALGORITHM__
#define TCAADDR 0x70

class WallFollow {
	private:
		/// Fields ///
		IRSensor** sensors;
		int selectedSensor_0;
		int selectedSensor_1;
		uint16_t threshold;
		bool debug;
		char buf[100];
		
		/// Methods ///
		void TCASELECT(uint8_t);
		bool checkSensors(int, int, uint16_t);
    bool LimitCheck(int, uint16_t);
		void printSensorData(int);
	public:
		/// Methods ///
		WallFollow(IRSensor**, bool);
		void SetSensorsToTrace(int, int);
		void Act();
		void Act(int, int);
		void Initialize(int, int, uint16_t);
};

#endif
