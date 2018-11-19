#include <iostream>

#ifdef PIBUILD
#include <WiringPi.h>
#else
#include "Arduino.h"
#endif 


#ifndef __PININTERFACE_INCLUDED__
#define __PININTERFACE_INCLUDED__

enum boardConfig {
	pi,
	arduino
};

class PinIO {
	private:
		void(*pMode) (int, int);
		void(*dWrite) (int, int);
		void(*dRead) (int);
	
	public:
		PinIO(boardConfig);
		void pinMode(int, int);
		void digitalWrite(int, int);
		void digitalRead(int);
		void writeOut();
};

#endif