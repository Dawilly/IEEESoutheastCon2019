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
    cout << "Waiting for remote system . . . " << arduino.readToken() << endl
         << endl;
    
    // Ping a token
    arduino.write("Hello!");
    cout << "Recieved token is " << arduino.readToken() << endl;
    
    // Ping an integer
    arduino.write(to_string(42));
    cout << "Received integer is " << arduino.readInt() << endl;
    
    // Ping a real number
    arduino.write(to_string(3.14159265359));
    cout << "Received real is " << arduino.readReal() << endl << endl;
    
    return 0;
}
