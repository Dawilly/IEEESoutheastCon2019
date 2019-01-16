// 
// 
// 

#include "SDS01A.h"


// Constructors
SDS01A::SDS01A() {
  
}

SDS01A::SDS01A(byte OutputPin) {
	this->OutputPin = OutputPin;
	this->readMode = raw;
}

SDS01A::SDS01A(byte OutputPin, ReadOperation readMode) {
	this->OutputPin = OutputPin;
	this->readMode = readMode;
}

void SDS01A::setReadMode(ReadOperation readMode) {
	this->readMode = readMode;
}

double SDS01A::scan() {
	RawDistance = analogRead(OutputPin);
	return RawDistance;
}
