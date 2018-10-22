#ifndef __HCSR04_H__
#define __HCSR04_H__
#include "Arduino.h"

enum ReadOperation
{
	raw,
	inches,
	centimeters
};

class HCSR04 {
	private:
		int TriggerPin;
		int EchoPin;
		int RefreshRate;
		ReadOperation ReadMode;
		long RawDistance;
		double distance;

	protected:
		void setPinMode();
		long Trigger();
		double ConvertValue(ReadOperation);

	public:
		HCSR04();
		HCSR04(int, int);
		HCSR04(int, int, ReadOperation);
		HCSR04(int, int, ReadOperation, int);
		void setReadMode(ReadOperation);
		double scan();
		double getLastScan();
		double getLastScan(ReadOperation);
		long getLastScanRaw();
		int getRefreshRate();
};

#endif
