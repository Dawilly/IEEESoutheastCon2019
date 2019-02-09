#ifndef __HCSR04__INCLUDED__
#define __HCSR04__INCLUDED__
#include <wiringPi.h>

class HCSR04 {
	private:
		int triggerPin;
		int echoPin;
		long lastReading;

		void getMeasurement();
		void setUp();
		long pulseIn();

	public:
		HCSR04();
		HCSR04(int, int);
		void Measure();
		double MeasureCm();
		double MeasureMm();
		double MeasureInches();
		double MeasureRaw();
};

#endif