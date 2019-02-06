#include "tof.h"
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>


// LED Pin - wiringPi pin 0 is BCM_GPIO 17.
// we have to use BCM numbering when initializing with wiringPiSetupSys
// when choosing a different pin number please use the BCM numbering, also
// update the Property Pages - Build Events - Remote Post-Build Event command 
// which uses gpio export for setup for wiringPiSetupSys
#define	LED	17

int main(void)
{
	int i = tofInit(1, 0x29, 1);
	int distance;
	int model;
	int revision;

	if (i != 1) {
		return -1;
	}

	printf("VL53L0X device successfully opened.\n");
	i = tofGetModel(&model, &revision);
	printf("Model ID - %d\n", model);
	printf("Revision ID - %d\n", revision);

	for (i = 0; i < 1200; i++) {
		distance = tofReadDistance();
		if (distance < 4096) printf("Distance: %d mm", distance);
		usleep(50000);
	}

	return 0;
}