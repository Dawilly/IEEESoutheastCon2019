#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <pthread.h>
#include <unistd.h>
#include "logger.h"
/*
P1  Name  gpio    used for
 2  5V    ---     5V
 6  GND   ---     Ground
24  CE0   8       Sonar echo
26  CE1   7       Sonar trigger
*/

#define SONAR_TRIGGER 23
#define SONAR_ECHO    17

/* forward prototypes */
void sonarTrigger(void);
void sonarEcho(int gpio, int level, uint32_t tick);

FILE* writer;
Logger* logger;
int param;

int main(int argc, char *argv[])
{
	if (gpioInitialise() < 0) return 1;
	gpioSetMode(SONAR_TRIGGER, PI_OUTPUT);
	gpioWrite(SONAR_TRIGGER, PI_OFF);
	gpioSetMode(SONAR_ECHO, PI_INPUT);

	writer = fopen(argv[1], "w");
	logger = createLogger(writer, 0);
	runLogger(logger);
	param = atoi(argv[2]);
	
	/* update sonar 20 times a second, timer #0 */
	gpioSetTimerFunc(0, 50, sonarTrigger); /* every 50ms */

	/* monitor sonar echos */
	gpioSetAlertFunc(SONAR_ECHO, sonarEcho);

	while (1) sleep(1);
	gpioTerminate();

	return 0;
}

void sonarTrigger(void)
{
	/* trigger a sonar reading */
	gpioWrite(SONAR_TRIGGER, PI_ON);
	gpioDelay(10); /* 10us trigger pulse */
	gpioWrite(SONAR_TRIGGER, PI_OFF);
}

void sonarEcho(int gpio, int level, uint32_t tick)
{
	char str[50];
	static uint32_t startTick, firstTick = 0;
	static uint32_t count = 0;
	int diffTick;
	if (!firstTick) firstTick = tick;
	if (level == PI_ON) {
		startTick = tick;
	}
	else if (level == PI_OFF) {
		diffTick = tick - startTick;
		
		double distance = 100*((diffTick/1000000.0)*340.29)/2;
		count++;
		snprintf(str, 50, "%lf", distance);
		addDataMessage(logger, param, count, str);
		printf("Distance: %lf cm\n", distance);
		
		//printf("%u %u\n", tick - firstTick, diffTick);
	}
	if (count == 50) {
		gpioDelay(5000);
		fclose(writer);
		exit(-1);
	}
}
