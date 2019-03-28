#include <unistd.h>
/*
 *directions:
 * forward = 1
 * backward = 2
 * right = 3
 * left = 4 
 */

void fwd_timed(int  speed, int time) {
	int direction = 1;
	//send direction and speed to arduino
	usleep(time);
}


void turnRight_timed(int speed, int time, double targetHeading) {
	int direction = 3;
	//send direction and speed and heading to arduino
	usleep(time);
}

void turnLeft_timed (int speed, int time, double targetHeading) {
	int direction = 4;
	//send direction, speed, and heading to arduino
	usleep(time);
}

