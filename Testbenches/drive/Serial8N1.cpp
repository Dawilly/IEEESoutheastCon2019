#include "Serial8N1.h"

// Input and Output
#include <iostream>
#include <unistd.h>

// Strings and Characters
#include <string>
#include <cctype>

// Sleeping
#include <chrono>
#include <thread>

// Serial port opening and configuration
#include <fcntl.h>
#include <termios.h>

// Constructor
Serial8N1::Serial8N1(const char *port, int speed) {
    /* OPEN THE SERIAL PORT */
    try {
        this->fd = ::open(port, O_RDWR | O_NOCTTY);
        if (this->fd == -1)
            throw port;
    }
    catch (const char *port) {
        std::cerr << "Serial8N1::Serial8N1(2) - Unable to open " << port
                  << std::endl;
        std::exit(EXIT_FAILURE);
    }

    /* CONFIGURE THE SERIAL PORT */
    struct termios options;
    tcgetattr(this->fd, &options);

    // Set Baud Rate
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // Set 8N1 Mode
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Enable Receiver and Set Local Mode
    options.c_cflag |= CLOCAL | CREAD;

    // Set Raw Input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Disable Processed Output
    options.c_oflag &= ~OPOST;

    // Disable Hardware Flow Control
    options.c_cflag &= ~CRTSCTS;

    // Disable Software Flow Control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Set Read Timeouts
    options.c_cc[VMIN] = 1;  // One character per packet
    options.c_cc[VTIME] = 0; // Wait indefinitely for packet

    // Set Options to Serial Port
    tcsetattr(this->fd, TCSANOW, &options);

    /* FLUSH SERIAL PORT */
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    tcflush(this->fd, TCIOFLUSH);
}

// Destructor
Serial8N1::~Serial8N1() {
    try {
        int result = ::close(this->fd);
        if (result < 0) throw this->fd;
    }
    catch (int fd) {
        std::cerr << "Serial8N1::~Serial8N1(0) - Unable to close file #"
                  << fd << std::endl;
    }
    ::close(this->fd);
}

// Reads a single character
char Serial8N1::read() {
    char ch;
    ::read(this->fd, &ch, 1);
    return ch;
}

// Tokens are strings that end in whitespace
std::string Serial8N1::readToken() {
    std::string output;
    
    char ch;
    while (::read(this->fd, &ch, 1) > 0) {
        if (std::isspace(ch)) break;
        output += ch;
    }
    
    return output;
}

// A line ends in a '\n' or "\r\n"
std::string Serial8N1::readLine() {
    std::string output;

    char ch;
    while (::read(this->fd, &ch, 1) > 0) {
        if (ch == '\n') break;
        if (ch == '\r') {
            // The next character must be a '\n', so process it
            ::read(this->fd, &ch, 1);
            break;
        }
        output += ch;
    }

    return output;
}

// Integers end in whitespace
int Serial8N1::readInt() {
    std::string buffer;
    try {
        char ch;
        while (::read(this->fd, &ch, 1) > 0) {
            if (std::isspace(ch)) break;
            // Integers contain only digits
            if (!std::isdigit(ch)) throw ch;
            buffer += ch;
        }
    }
    catch (char offending) {
        std::cerr << "Serial8N1::readInt(0) - Offending character <"
                  << offending << ">" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return std::stoi(buffer);
}

// Real numbers end in whitespace
double Serial8N1::readReal() {
    std::string buffer;
    try {
        char ch;
        bool hasDecimal = false;
        while (::read(this->fd, &ch, 1) > 0) {
            if (std::isspace(ch)) break;
            if (!std::isdigit(ch)) {
                if (ch == '.') {
                    // Real numbers only have one decimal point
                    if (hasDecimal) throw ch;
                    else hasDecimal = true;
                }
                // Real numbers contain digits or a single decimal point
                else throw ch;
            }
            buffer += ch;
        }
    }
    catch (char offending) {
        std::cerr << "Serial8N1::readReal(0) - Offending character <"
                  << offending << ">" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return std::stod(buffer);
}

// All transmitted messages must be strings and end in a whitespace
void Serial8N1::write(std::string message) {
    message += '\n';
    try {
        int nbytes = (int) ::write(fd, message.c_str(), message.size());
        if (nbytes < 0) throw message;
    }
    catch (std::string message) {
        std::cerr << "Serail8N1::write(1) - Message  \"" << message
                  << "\" failed to send" << std::endl;
    }
}
