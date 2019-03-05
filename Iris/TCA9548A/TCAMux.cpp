#include <iostream>
#include <linux/i2c-dev.h>
#include <cstdio>
#include "TCAMux.h"

using namespace std;

TCAMux::TCAMux() {
	this->status = -1;
	this->activeInput = -1;
	this->dataWrite = 0x00;
}

void TCAMux::Initialize() {
	char buffer[15] = { 0 };
	snprintf(buffer, 15, "/dev/i2c-1");
	status = open(buffer, O_RDWR);
	if ()
	if (ioctl(status, I2C_SLAVE, 0x70) < 0) {
		cout << "TCAMux Initalization Failed" << endl;
		status = -2;
	} else {
		cout << "TCAMux Initalized." << endl;
		status = 0;
		Switch(0);
	}
	return;
}

void TCAMux::Switch(int value) {
	if (status != 0) {
		cout << "Cannot switch selector." << endl;
		return;
	} 
	//mux->data_write[0] = 1 << inputNo;
	dataWrite = 1 << value;
	//write(mux->muxStatus, mux->data_write, 1);
	write(status, &dataWrite, 1);
	activeInput = value;
	cout << "Mux set to: " << activeInput << endl;
	cout << "Status: " << status << endl;
	cout << "Datawrite: " << dataWrite << endl;
	return;
}
