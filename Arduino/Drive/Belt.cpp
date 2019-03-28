#include "Arduino.h"
#include "Motors.h"
#include "Belt.h"

#define BELT_SPEED 255
#define QUICK_RANGE 600
#define SLOW_RANGE 20

const static long LEVEL_TO_ENCODER[] = {
    POSITION_WAIT,
    POSITION_SORT1,
    POSITION_SORT2};

// Constructor
Belt::Belt(uint8_t MPWMpin, uint8_t FPIN, uint8_t RPIN, uint8_t pin1,
           uint8_t pin2) : m(MPWMpin, FPIN, RPIN, pin1, pin2)
{
    this->m.run(STOP);
    this->m.setSpeed(0);
    this->current_encoder = this->m.getPosition();
}

// Deconstuctor
Belt::~Belt() {}

// Moves motor until it reaches the position
// Returns true if the position is reached, and false otherwise
bool Belt::moveTo(BeltPosition x) {
    // Determine desired position
    long desired_encoder = LEVEL_TO_ENCODER[(int) x];

    // Initialize current position, difference in position, and direction
    this->current_encoder = this->m.getPosition();
    long diff = desired_encoder - this->current_encoder;
    int dir = (diff > 0) ? FORWARD : BACKWARD;

    // Quiclky move (do not change direction) to desired encoder position
    while ((dir == FORWARD) ? diff > QUICK_RANGE : diff < -QUICK_RANGE) {
        this->m.run(dir);
        this->m.setSpeed(BELT_SPEED);
        if (!this->moveMotor(1, 1000)) {
            // Motor has stalled
            this->stopMotor();
            return false;
        }
        this->current_encoder = this->m.getPosition();
        diff = desired_encoder - this->current_encoder;
    }
    this->stopMotor();

    // Update current position, difference in position, and direction
    this->current_encoder = this->m.getPosition();
    diff = desired_encoder - this->current_encoder;
    dir = (diff > 0) ? FORWARD : BACKWARD;

    // Slowly move to desired encoder position
    while (abs(diff) > SLOW_RANGE) {
        this->m.run(dir);
        this->m.setSpeed(BELT_SPEED);
        if (!this->moveMotor(5, 1000)) {
            // Motor has stalled
            this->stopMotor();
            return false;
        }
        this->stopMotor();
        this->current_encoder = this->m.getPosition();
        diff = desired_encoder - this->current_encoder;
        dir = (diff > 0) ? FORWARD : BACKWARD;
    }

    // Belt motor successfully reached position
    return true;
}

// Update current position and return its value
long Belt::getPosition() {
    return (this->current_encoder = this->m.getPosition());
}

// Stop the motor and move until it does not move 1 encoder tick for 50 ms
void Belt::stopMotor() {
    this->m.run(STOP);
    this->m.setSpeed(0);
    for (; this->moveMotor(1, 50);
         this->current_encoder = this->m.getPosition());
}

// Moves motor until reaching the number of encoder ticks, or a timeout occurs
// Returns true if motor moved number of encoder ticks, and false otherwise
bool Belt::moveMotor(int encoder_ticks, unsigned long timeout) {
    unsigned long start = millis();
    while (abs(this->current_encoder - this->m.getPosition()) < encoder_ticks) {
        if (millis() - start > timeout) return false;
    }
    return true;
}
