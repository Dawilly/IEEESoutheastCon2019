#include <iostream>
#include "Serial8N1.h"

using namespace std;

int main(int argc, char **argv) {
    // Check number of arguments
    if (argc < 2) {
        cerr << "Too few arguments!" << endl;
        return -1;
    }

    // Open serial port
    Serial8N1 arduino(argv[1], 9600);
    //cout << "Waiting for remote system . . . " << arduino.readToken() << endl
         //<< endl;
    
    // Drive straight for 36 inches
    arduino.write(to_string(1) + ' ' + to_string(36));
    return 0;
}
