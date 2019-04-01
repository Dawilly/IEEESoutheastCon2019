#include <cstdio>
#include <string>

#ifndef __SERIAL8N1_INCLUDED__
#define __SERIAL8N1_INCLUDED__

class Serial8N1 {
    private:
        int fd;

    public:
        Serial8N1(const char *port, int open_flags);
        ~Serial8N1();

        // Receivers (all reads must end in whitespace)
        char read();
        std::string readToken();
        std::string readLine();
        int readInt();
        double readReal();

        // Transmitter
        void write(std::string message);
};

#endif
