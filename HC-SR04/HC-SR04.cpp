#include "HC-SR04.h"

// Constructors

/// <summary>
/// Default Constructor for the HCSR04 class to interact with an HC-SR04 sensor.
/// Trigger Pin and Echo Pin will be set to 53 and 50 respectively. Refresh Rate
/// is set to 500. ReadMode is set to raw.
/// </summary>
HCSR04::HCSR04() {
	TriggerPin = 53;
	EchoPin = 50;
	this->RefreshRate = 500;
	this->setReadMode(raw);
	this->setPinMode();
}

/// <summary>
/// Constructor for the HCSR04 class to interact with an HC-SR04 sensor. User sets the 
/// Trigger and Echo pin. Refresh Rate is set to 500. Readmode is set to raw.
/// </summary>
/// <param name="TriggerPin">The Arduino pin number that is connected to the Trigger Pin of the HC-SR04.</param>
/// <param name="EchoPin">The Arduino pin number that is connected to the Echo Pin of the HC-SR04.</param>
HCSR04::HCSR04(int TriggerPin, int EchoPin) {
	this->TriggerPin = TriggerPin;
	this->EchoPin = EchoPin;
	this->RefreshRate = 500;
	this->setPinMode();
}

/// <summary>
/// Constructor for the HCSR04 class to interact with an HC-SR04 sensor. User sets the 
/// Trigger pin, Echo pin, and Readmode. Refresh Rate is set to 500.
/// </summary>
/// <param name="TriggerPin">The Arduino pin number that is connected to the Trigger Pin of the HC-SR04.</param>
/// <param name="EchoPin">The Arduino pin number that is connected to the Echo Pin of the HC-SR04.</param>
/// <param name="ReadMode">Sets what unit of measurement the scan function of the HCSR04 class should return with.</param>
HCSR04::HCSR04(int TriggerPin, int EchoPin, ReadOperation ReadMode) {
	this->TriggerPin = TriggerPin;
	this->EchoPin = EchoPin;
	this->RefreshRate = 500;
	this->setReadMode(ReadMode);
	this->setPinMode();
}

/// <summary>
/// Constructor for the HCSR04 class to interact with an HC-SR04 sensor. User sets the 
/// Trigger pin, Echo pin, Read mode and Refresh Rate.
/// </summary>
/// <param name="TriggerPin">The Arduino pin number that is connected to the Trigger Pin of the HC-SR04.</param>
/// <param name="EchoPin">The Arduino pin number that is connected to the Echo Pin of the HC-SR04.</param>
/// <param name="ReadMode">Sets what unit of measurement the scan function of the HCSR04 class should return with.</param>
/// <param name="RefreshRate">Sets the amount of time the Arduino should wait before the next reading (in milliseconds).</param>
HCSR04::HCSR04(int TriggerPin, int EchoPin, ReadOperation ReadMode, int RefreshRate) {
	this->TriggerPin = TriggerPin;
	this->EchoPin = EchoPin;
	this->RefreshRate = RefreshRate;
	this->setReadMode(ReadMode);
	this->setPinMode();
}

// Public Methods

/// <summary>
/// Uses the HC-SR04 functional purpose to detect (in a unit of distance) how far an object is.
/// </summary>
/// <returns>
/// (Double) The distance the HC-SR04 detects some type of object. The unit of measurement this value provides can be set with setReadMode(ReadOperation).
/// <returns>
double HCSR04::scan() {
	RawDistance = Trigger();
	distance = ConvertValue(this->ReadMode);
	return distance;
}

/// <summary>
/// Sets the unit of measurement (the length of whatever object it detects) the sensor will provide when used for its intended purpose.
/// </summary>
/// <returns>
/// Void (Nothing)
/// <returns>
/// <param name="ReadMode">The unit of measurement the sensor should return (inches, centimeters, etc)</param>
void HCSR04::setReadMode(ReadOperation ReadMode) {
	this->ReadMode = ReadMode;
	return;
}

/// <summary>
/// Get the last scan operation's returned value, in it's raw value form.
/// </summary>
/// <returns>
/// (Long) The raw distance value.
/// <returns>
long HCSR04::getLastScanRaw() {
	return this->RawDistance;
}

/// <summary>
/// Get the last scan operation's returned value, in converted form.
/// </summary>
/// <returns>
/// (Double) The value from the previous scan operation.
/// <returns>
double HCSR04::getLastScan() {
	return this->distance;
}

/// <summary>
/// Get the last scan operation's returned value. Allows the user to convert to another unit of measurement,
/// regardless of the instance's ReadMode value.
/// </summary>
/// <returns>
/// (Double) The value from the previous scan operation, converted into another unit of measurement.
/// <returns>
/// <param name="convertReadMode">The unit of measurement the sensor should return (inchest, centimeters, etc).</param>
double HCSR04::getLastScan(ReadOperation convertReadMode) {
	return this->ConvertValue(convertReadMode);
}

/// <summary>
/// Gets the stored refresh rate.
/// </summary>
/// <returns>
/// (Int) The refresh rate (in milliseconds).
/// <returns>
int HCSR04::getRefreshRate() {
	return this->RefreshRate;
}

// Protected Methods

/// <summary>
/// Sets the pin number associated to Trigger and Echo to Output and Input, respectively.
/// </summary>
/// <returns>
/// (Void) Nothing
/// <returns>
void HCSR04::setPinMode() {
	pinMode(TriggerPin, OUTPUT);
	pinMode(EchoPin, INPUT);
	return;
}

/// <summary>
/// Internal Mechanic - to using the HC-SR04 to find the proximity of anything within the field. 
/// </summary>
/// <returns>
/// (Long) The raw distance value.
/// <returns>
long HCSR04::Trigger() {
	long reading;
	digitalWrite(this->TriggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(this->TriggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(this->TriggerPin, LOW);
	reading = pulseIn(EchoPin, HIGH);
	return reading;
}

/// <summary>
/// Internal Mechanicsm - Converts the value from raw to the desired units of measurement.
/// </summary>
/// <returns>
/// (Double) The distance value in the desired unit of measurement.
/// <returns>
/// <param name="convertReadMode">The unit of measurement the sensor should return (inchest, centimeters, etc).</param>
double HCSR04::ConvertValue(ReadOperation AsReadMode) {
	double results;

	switch (AsReadMode) {
		case inches:
			results = RawDistance / 74.0 / 2.0;
			break;
		case centimeters:
			results = RawDistance / 29.0 / 2.0;
			break;
		default:
			results = -1;
			break;
	}

	return results;
}