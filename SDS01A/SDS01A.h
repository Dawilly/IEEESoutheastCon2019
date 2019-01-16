// SDS01A.h

#ifndef __SDS01A_INCLUDED__
#define __SDS01A_INCLUDED__

#ifdef PIBUILD
#include <wiringPi.h>
#else
#include "Arduino.h"
#endif

/// Enumeration Defintion. For Units of Measurement.
enum ReadOperation
{
	raw,
	inches,
	centimeters
};

class SDS01A {
	private:
		byte OutputPin;
		long RawDistance;
		double distance;
		ReadOperation readMode;

	protected:
		double ConvertValue(ReadOperation);

	public:
		SDS01A();
		SDS01A(byte);
		SDS01A(byte, ReadOperation);
		void setReadMode(ReadOperation);
		double scan();
};

#endif

