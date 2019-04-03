#include <iostream>
#include <fstream>
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
        to_string(6) + ' ' + to_string(30.0),
        to_string(8) + ' ' + to_string(270.0),
        to_string(1) + ' ' + to_string(10.0),
        to_string(8) + ' ' + to_string(0.0),
        to_string(1) + ' ' + to_string(13.0),
        to_string(8) + ' ' + to_string(315.0),
        to_string(1) + ' ' + to_string(44.5),
        to_string(8) + ' ' + to_string(225.0),
        to_string(1) + ' ' + to_string(44.5),
        to_string(8) + ' ' + to_string(135.0),
        to_string(1) + ' ' + to_string(44.5),
        to_string(8) + ' ' + to_string(25.72),
        to_string(1) + ' ' + to_string(37.5),
        to_string(8) + ' ' + to_string(315.0),
        to_string(1) + ' ' + to_string(32.0),
        to_string(8) + ' ' + to_string(225.0),
        to_string(1) + ' ' + to_string(32.0),
        to_string(8) + ' ' + to_string(135.0),
        to_string(1) + ' ' + to_string(32.0),
        to_string(8) + ' ' + to_string(29.6),
        to_string(1) + ' ' + to_string(25.0),
        to_string(8) + ' ' + to_string(315.0),
        to_string(1) + ' ' + to_string(19.0),
        to_string(8) + ' ' + to_string(225.0),
        to_string(1) + ' ' + to_string(19.0),
        to_string(8) + ' ' + to_string(135.0),
        to_string(1) + ' ' + to_string(19.0),
        to_string(8) + ' ' + to_string(90.0),
        to_string(1) + ' ' + to_string(43.0),
        to_string(8) + ' ' + to_string(180.0),
        to_string(1) + ' ' + to_string(27.0)};
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

    // Wait until final command is done processing and finish
    while (arduino_ready != true);
    gpioTerminate();

    // Create a log file for the run
    ofstream log;
    log.open("log.txt");
    string buffer = arduino.readLine();
    for (int i = 0; i < size; buffer = arduino.readLine()) {
        if (buffer == "EOL") i++;
        else log << buffer << endl;
    }
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
