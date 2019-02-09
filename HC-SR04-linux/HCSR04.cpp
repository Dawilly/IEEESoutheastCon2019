#include "HCSR04.hpp"
#include <wiringPi.h>
#include <chrono>
#include <cmath>

HCSR04::HCSR04() {
	triggerPin = 27;
	echoPin = 28;
	setUp();
}

HCSR04::HCSR04(int triggerPin, int echoPin) {
	this->triggerPin = triggerPin;
	this->echoPin = echoPin;
	setUp();
}

void HCSR04::Measure() {
	return;
}

double HCSR04::MeasureCm() {
	return 0.0;
}

double HCSR04::MeasureMm() {
	return 0.0;
}

double HCSR04::MeasureInches() {
	return 0.0;
}

double HCSR04::MeasureRaw() {
	return 0.0;
}

void HCSR04::getMeasurement() {
	using namespace std;

	//Ensure triggerPin is at 0
	digitalWrite(triggerPin, false);
	//Wait 2 microseconds
	delayMicroseconds(2);
	//Output 1 to triggerPin
	digitalWrite(triggerPin, true);
	//Wait 10 microseconds
	delayMicroseconds(10);
	//Output 0 to triggerPin
	digitalWrite(triggerPin, false);

	//Measure the pulse of the echo
	lastReading = pulseIn() * 17150;
	
	return;
}

void HCSR04::setUp() {
	pinMode(triggerPin, OUTPUT);
	pinMode(echoPin, INPUT);
	lastReading = 0;
	return;
}

//Get the Pulse Durtaion
long HCSR04::pulseIn() {
	using namespace std::chrono;

	//Clock the beginning time
	auto begin = high_resolution_clock::now();
	auto end = high_resolution_clock::now();

	//while echo is low
	while (digitalRead(echoPin) == 0) {
		//Clock the beginning time
		begin = high_resolution_clock::now();
	}
	//While echo is high
	while (digitalRead(echoPin) == 1) {
		//Clock the end time
		end = high_resolution_clock::now();
	}
	//Duration = end - begin
	auto duration = end - begin;
	//Convert to microseconds
	auto conv = duration.time_since_epoch();
	//Return
	return conv.count();
}