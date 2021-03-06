#include <iostream>
#include <chrono>
#include <thread>
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
    
    // Initialize a list of commands
    string commands[] = {
        to_string(1) + ' ' + to_string(30.0),
        to_string(5) + ' ' + to_string(1)};
    int size = sizeof(commands) / sizeof(*commands);

    // Execute commands when the arduino is ready to receive them
    int i = 0;
    while (i < size) {
        if (arduino_ready == true) {
            arduino.write(commands[i]);
            i++;
            arduino_ready = false;
        }
    }

    // Hold block for 3 seconds, then deposit block
    while (arduino_ready != true);
    this_thread::sleep_for(chrono::seconds(3));
    arduino.write(to_string(5) + ' ' + to_string(0));

    // Wait until final command is done processing and finish
    while (arduino_ready != true);
    gpioTerminate();
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
