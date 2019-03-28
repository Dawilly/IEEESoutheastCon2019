#include <iostream>
#include <fstream>
#include <time.h>
#include <chrono>
#include "logger.hpp"
#include "message.hpp"

using namespace std;

void Logger::Bootstrapper(Logger* instance) {
	message nextMsg;
	
	while(instance->running) {
		if (instance->Messages.size() > 0) {
			nextMsg = instance->Messages.front();
			instance->WriteMessage(instance->file, nextMsg);
			instance->Messages.pop();
		} else {
			this_thread::sleep_for(chrono::milliseconds(200));
		}
	}
	
	//Ending, catch up;
	while(instance->Messages.size() > 0) {
		nextMsg = instance->Messages.front();
		instance->WriteMessage(instance->file, nextMsg);
		instance->Messages.pop();
	}
	
	return;
}

void Logger::WriteMessage(ofstream& fp, message msg) {
	fp << msg.timestamp << delimiter;
	
	if (msg.vType == Str) {
		string *sp = static_cast<string*>(msg.value);
		fp << (*sp);
		delete sp;
	} else if (msg.vType == Integer) {
		int *ip = static_cast<int*>(msg.value);
		fp << (*ip);
		delete ip;
	} else if (msg.vType == Real) {
		double *dp = static_cast<double*>(msg.value);
		fp << (*dp);
		delete dp;
	}
	
	if (msg.mType != None) {
		fp << delimiter << msg.mType;
	}
	fp << endl;
}

Logger::Logger(string filename, string delim) {
	running = false;
	delimiter = delim;
	file.open(filename);
	if (!file.is_open()) {
		cout << "Unable to open file: " << filename << endl;
		exit(-1);
	}
}

void Logger::AddMessage(string* v, messageType mType) {
	AddMessage(v, mType, Str);
	return;
}

void Logger::AddMessage(int* v, messageType mType) {
	AddMessage(v, mType, Integer);
	return;
}

void Logger::AddMessage(double* v, messageType mType) {
	AddMessage(v, mType, Real);
	return;
}

void Logger::AddMessage(void* v, messageType mType, valueType vType) {
	message newMsg;
	newMsg.timestamp = time(0);	
	newMsg.value = v;
	newMsg.vType = vType;
	newMsg.mType = mType;
	Messages.push(newMsg);
	return;
}

void Logger::Run() {
	cout << "Starting Logger..." << endl;
	running = true;
	thread(Bootstrapper, this).detach();
	return;
}

void Logger::Stop() {
	running = false;
}