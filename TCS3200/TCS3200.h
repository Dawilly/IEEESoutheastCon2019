
#ifndef __TCS3200_H__
#define __TCS3200_H__
#include "Arduino.h"

enum pdType {
	red = 0,
	blue = 1,
	green = 2,
	clear = 3
};

typedef struct PhotoDiode {
	int s2pin;
	int s3pin;
} PhotoDiode;

class TCS3200 {
	private: 
		int SPins[4];
		int OutputPin;

		PhotoDiode pinSettings[4] = {
			{ LOW, LOW }, //red
			{ LOW, HIGH}, //blue
			{ HIGH,HIGH}, //green
			{ HIGH,LOW }, //clear
		};

		int redValue = 0;
		int greenValue = 0;
		int blueValue = 0;

		void scanColor(pdType, int*);

	protected:
		void setPins();
		void scanRed();
		void scanGreen();
		void scanBlue();
		//void scanClear();

	public:
		TCS3200();
		TCS3200(int*, int);
		TCS3200(int, int, int, int, int);
		void scan();
		//String toString();
		void printResults();
};

#endif