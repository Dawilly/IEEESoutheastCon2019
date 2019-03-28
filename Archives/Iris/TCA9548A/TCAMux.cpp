#include <iostream>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "TCAMux.h"

using namespace std;

TCAMux::TCAMux() {
	this->status = -1;
	this->activeInput = -1;
	this->dataWrite = new char[1];
	this->dataWrite[0] = 0x00;
}

void TCAMux::Initialize() {
	char* filename = (char*)"/dev/i2c-1";
	if ((fd = open(filename, O_RDWR)) < 0) {
		cout << "TCAMux: Unable to open." << endl;
		cout << "Error: " << strerror(errno) << endl;
		status = -1;
		return;
	}
	
	if (ioctl(fd, I2C_SLAVE, 0x70) < 0) {
		cout << "TCAMUX: Failed to acquire buss access." << endl;
		cout << "Error: " << strerror(errno) << endl;
		status = -2;
	}
	status = 0;
}

void TCAMux::Switch(int value) {
	if (status != 0) {
		cout << "Cannot switch selector." << endl;
		return;
	} 
	
	dataWrite[0] = 1 << value;
	
	if (write(fd, dataWrite, 1) < 0) {
		cout << "Unable to switch pins." << endl;	
	}
	activeInput = value;
	cout << "Mux set to: " << activeInput << endl;
	cout << "Status: " << status << endl;
	cout << "Datawrite: " << dataWrite << endl;
	return;
}
