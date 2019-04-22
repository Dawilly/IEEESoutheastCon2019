#include "Arduino.h"
#include "Motors.h"

#ifndef __BELT_INCLUDED__
#define __BELT_INCLUDED__

typedef enum belt_position {
    WAITING_LEVEL = 0,
    SORTING_LEVEL_1,
    SORTING_LEVEL_2
} BeltPosition;

class Belt {
    // Encoder positions for each level. The waiting level is always zero.
    #define POSITION_WAIT 0
    #define POSITION_SORT1 4900
    #define POSITION_SORT2 4900

    public:
        Belt(uint8_t MPWMpin, uint8_t FPIN, uint8_t RPIN, uint8_t pin1,
             uint8_t pin2);
        ~Belt();
        bool moveTo(BeltPosition x);
        long getPosition();
    private:
        Motor m;
        long current_encoder;
        // Helper methods
        void stopMotor();
        bool moveMotor(int encoder_ticks, unsigned long timeout);
};

#endif
