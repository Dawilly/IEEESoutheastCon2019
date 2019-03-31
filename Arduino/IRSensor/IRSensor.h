#include <Adafruit_VL53L0X.h>
#include <Wire.h>

#ifndef __IRSENSOR__INCLUDED__
#define __IRSENSOR__INCLUDED__

enum SensorPlacement {
	LeftBottom = 0,
	LeftTop = 1,
	RightTop = 2,
	RightBottom = 3,
	
	FrontLeft = 0,
	FrontRight = 0,
	BackLeft = 0,
	BackRight = 0
};

class IRSensor {
	
	private:
		Adafruit_VL53L0X* sensor;
		VL53L0X_RangingMeasurementData_t* measurement;
		bool debug;
		int offset;
		double angleOffset;
		void SetUp(bool, int);
		
	public:
		bool success;
		
		IRSensor();
		IRSensor(bool);
		IRSensor(int, bool);
		~IRSensor();
		int readRange();
		int readRangeAsInt();
		int lastReading();
		int lastReadingAsInt();
		int RangeStatus();
		void adjustOffset(int);
};

#endif
