#ifndef __HCSR04__INCLUDED__
#define __HCSR04__INCLUDED__
#include <wiringPi.h>

class HCSR04 {
	private:
		int triggerPin;
		int echoPin;
		double lastReading;
		long now;
		volatile long start;
		volatile long end;

		void getMeasurement(int);
		void setUp();
		void pulseLength();

	public:
		HCSR04();
		HCSR04(int, int);
		double MeasureCm();
		double MeasureMm();
		double MeasureInches();
		double MeasureRaw();
};

#endif