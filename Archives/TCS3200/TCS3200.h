
#ifndef __TCS3200_H__
#define __TCS3200_H__

#ifdef PIBUILD
#include <wiringPi.h>
#else
#include "Arduino.h"
#endif

enum pdType {
	red = 0,
	blue = 1,
	green = 2,
	clear = 3
};

struct PhotoDiode {
	int s2pin;
	int s3pin;
};

template <typename T>
struct Data {
	T red;
	T green;
	T blue;
	T clear;
};

class TCS3200 {
	private: 
		int SPins[4];
		int OutputPin;
		int LEDPin;
		int sampleSize = 10;
		bool enableLED;

		PhotoDiode pinSettings[4] = {
			{ LOW, LOW }, //red
			{ LOW, HIGH}, //blue
			{ HIGH,HIGH}, //green
			{ HIGH,LOW }, //clear
		};

		Data<float> rawData;
		Data<int> colorData;

		Data<float> _darkcal;
		Data<float> _whitecal;

		void scanColor(pdType, float*);
		void calibrateBlack();
		void calibrateWhite();
		int calculateRGB(float, float, float);
		void printRaw(Data<float>);

	protected:
		void initalize();
		void setPins();
		void scanRaw();
		void scanRaw(Data<float>);
		void convertRaw2RGB(Data<float>);

		template <typename T>
		void clearData(Data<T>);

	public:
		TCS3200(int*, int);
		TCS3200(int, int, int, int, int);
		TCS3200(int, int, int, int, int, int);
		void scan();
		void printResults(int);
		void calibration();
};

#endif