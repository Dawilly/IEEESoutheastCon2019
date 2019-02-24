#include "HCSR04.hpp"
#include <wiringPi.h>
#include <chrono>

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

double HCSR04::MeasureCm() {
	double results;
	getMeasurement(30000);
	results = 100*((lastReading/1000000.0)*340.29)/2;
	return results;
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

void HCSR04::getMeasurement(int timeout) {
	using namespace std;
	
	//Ensure triggerPin is at 0
	digitalWrite(triggerPin, LOW);
	//Wait 2 microseconds
	delayMicroseconds(2);
	//Output 1 to triggerPin
	digitalWrite(triggerPin, HIGH);
	//Wait 10 microseconds
	delayMicroseconds(10);
	//Output 0 to triggerPin
	digitalWrite(triggerPin, LOW);
	
	now = micros();
	
	while(digitalRead(echoPin) == LOW && (micros() - now < timeout)) {
		pulseLength();
	}
	
	lastReading = end - start;
	
	return;
}

void HCSR04::setUp() {
	pinMode(triggerPin, OUTPUT);
	pinMode(echoPin, INPUT);
	lastReading = 0;
	return;
}

//Get the Pulse Durtaion
void HCSR04::pulseLength() {
	start = micros();
	while (digitalRead(echoPin) == HIGH);
	end = micros();
}