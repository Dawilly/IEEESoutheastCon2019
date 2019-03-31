#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "IRSensor/IRSensor.h"

#ifndef __WALLFOLLOW_ALGORITHM__
#define __WALLFOLLOW_ALGORITHM__
#define TCAADDR 0x70

class WallFollow {
	private:
		/// Fields ///
		IRSensor** sensors;
		int selectedSensor_0;
		int selectedSensor_1;
		int threshold;
		int val1;
		int val0;
		bool debug;
		char buf[100];
		
		/// Methods ///
		void TCASELECT(uint8_t);
		bool checkForAdjustment(int, int, int);
		void printSensorData(int);
	public:
		/// Methods ///
		WallFollow(IRSensor**, bool);
		void SetSensorsToTrace(int, int);
		int Act();
		int Act(int, int);
		void Initialize(int, int, int);
		int FollowingLeftOrRight();
};

#endif
