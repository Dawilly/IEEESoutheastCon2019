#include <iostream>
#include <pigpio.h>

#define PIN 4

using namespace std;

void handle_interrupt(int gpio, int level, uint32_t tick);

int main(void) {
    // Set up GPIO 4 as "interrupt"
    gpioInitialise();
    gpioSetMode(PIN, PI_INPUT);
    gpioSetPullUpDown(PIN, PI_PUD_UP);

    // Pass function to handle interrupt at rising and falling edges
    gpioSetISRFunc(PIN, EITHER_EDGE, 0, handle_interrupt);

    // Consume CPU until interrupt occurs
    while(true);
    return 0;
}

// Function to handle change in level of GPIO 4 
void handle_interrupt(int gpio, int level, uint32_t tick) {
    cout << "Pin " << gpio << " caused interrupt of level " << level
         << " at time " << tick << endl;
}
