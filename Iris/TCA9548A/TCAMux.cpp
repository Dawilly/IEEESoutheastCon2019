#include <iostream>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "TCAMux.h"

using namespace std;

TCAMux::TCAMux() {
	this->status = -1;
	this->activeInput = -1;
	this->dataWrite = new char[1];
	this->dataWrite[0] = 0x00;
}

void TCAMux::Initialize() {
	char buffer[15] = { 0 };
	snprintf(buffer, 15, "/dev/i2c-1");
	status = open(buffer, O_RDWR);
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
	dataWrite[0] = 1 << value;
	//write(mux->muxStatus, mux->data_write, 1);
	if (write(status, dataWrite, 1) < 0) {
		cout << "Unable to switch pins." <<	endl;
	}
	activeInput = value;
	cout << "Mux set to: " << activeInput << endl;
	cout << "Status: " << status << endl;
	cout << "Datawrite: " << dataWrite << endl;
	return;
}

void TCAMux::scanI2CBus(char from_addr, char to_addr) {
	char data = 0; // not used, just a ptr to feed to twi_writeTo()
	for (char addr = from_addr; addr <= to_addr; addr++) {
		//char rc = twi_writeTo(addr, &data, 0, 1, 0);
		//write(fd, buf, len + 1)
		if (rc == 0) {
			cout << "Device Found at address: " << endl;
			Serial.printl("device found at address ");
			Serial.println(addr, DEC);
		}
	}
}
