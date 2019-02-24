#include <wiringPi.h>
#include <iostream>
#include "HCSR04.hpp"

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.
// we have to use BCM numbering when initializing with wiringPiSetupSys
// when choosing a different pin number please use the BCM numbering, also
// update the Property Pages - Build Events - Remote Post-Build Event command 
// which uses gpio export for setup for wiringPiSetupSys
#define	TRIGGER	23
#define ECHO 17

int main(void)
{
	wiringPiSetupSys();

	HCSR04 sensor(TRIGGER, ECHO);

	for (int i = 1; i <= 50; i++) {
		std::cout << "The distance is: " << sensor.MeasureCm() << " cm" << std::endl;
	}

	return 0;
}