#include <iostream>
#include <chrono>
#include <thread>
#include <pigpio.h>

#define PIN 4

using namespace std;

void handle_interrupt(int gpio, int level, uint32_t tick);

int main(void) {
    gpioInitialise();
    gpioSetMode(PIN, PI_INPUT);
    gpioSetPullUpDown(PIN, PI_PUD_UP);
    gpioSetISRFunc(PIN, EITHER_EDGE, 0, handle_interrupt);
    
    while(true);
    
    gpioTerminate();
    return 0;
}

void handle_interrupt(int gpio, int level, uint32_t tick) {
    cout << "Pin " << gpio << " caused interrupt of level " << level
         << " at time " << tick << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
}
