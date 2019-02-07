#include "HCSR04.hpp"
#include <wiringPi.h>
#include <chrono>

using namespace std;

HCSR04::HCSR04() {
	triggerPin = 27;
	echoPin = 28;
	lastReading = 0.0;
}

HCSR04::HCSR04(int triggerPin, int echoPin) {
	this->triggerPin = triggerPin;
	this->echoPin = echoPin;
	lastReading = 0.0;
}

void HCSR04::Measure() {

}

double HCSR04::MeasureCm() {

}

double HCSR04::MeasureMm() {

}

double HCSR04::MeasureInches() {

}

double HCSR04::MeasureRaw() {

}

void HCSR04::getMeasurement() {
	chrono::high_resolution_clock::
}