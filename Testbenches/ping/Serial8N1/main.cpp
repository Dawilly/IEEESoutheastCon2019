#include <iostream>
#include <pigpio.h>
#include "Serial8N1.h"

#define PIN 18

using namespace std;

void handle_interrupt(int gpio, int level, uint32_t tick, void *flag);

int main(int argc, char **argv) {
    // Check number of arguments
    if (argc < 2) {
        cerr << "Too few arguments!" << endl;
        return -1;
    }

    // Set up GPIO 18 as "interrupt" to indicate arduino is ready for a message
    gpioInitialise();
    gpioSetMode(PIN, PI_INPUT);
    gpioSetPullUpDown(PIN, PI_PUD_UP);

    // Pass function to handle interrupt at rising and falling edges
    bool arduino_ready = true;
    gpioSetISRFuncEx(PIN, EITHER_EDGE, 0, handle_interrupt, &arduino_ready);

    // Open serial port
    Serial8N1 arduino(argv[1], 9600);

    // Ping a token
    arduino.write("Hello!");
    arduino_ready = false;
    while (!arduino_ready);
    cout << "Recieved token is " << arduino.readToken() << endl;

    // Ping an integer
    arduino.write(to_string(42));
    arduino_ready = false;
    while (!arduino_ready);
    cout << "Received integer is " << arduino.readInt() << endl;

    // Ping a real number
    arduino.write(to_string(3.14159265359));
    arduino_ready = false;
    while (!arduino_ready);
    cout << "Received real is " << arduino.readReal() << endl << endl;

    return 0;
}

// Function to handle change in level of GPIO 18
void handle_interrupt(int gpio, int level, uint32_t tick, void *flag) {
    // At interrupt rising edge, arduino is ready for a new message
    if (level == 1) {
        cout << "GPIO " << gpio
             << ": Arduino is ready to receive a command at time " << tick
             << "." << endl;
        (*(bool *)flag) = true;
    }
    // At falling edge, arduino has begun processing a command
    else {
        cout << "GPIO " << gpio
             << ": Arduino has begun processing a command at time " << tick
             << "." << endl;
    }
}
