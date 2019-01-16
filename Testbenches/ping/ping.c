#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define BUFFER_SIZE 128

char *readLine(int);

int main() {
    /* OPEN SERIAL PORT */
    int fd;
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        printf("Unable to open /dev/ttyACM0.\n");
        return -1;
    } else {
        printf("Opened /dev/ttyASM0 successfully!\n");
    }

    /* CONFIGURE SERIAL PORT */
    struct termios options;
    tcgetattr(fd, &options);

    // Set Baud Rate
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

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
    tcsetattr(fd, TCSANOW, &options);

    /* FLUSH SERIAL PORT */
    usleep(10000);
    tcflush(fd, TCIOFLUSH);

    /* WAIT FOR REMOTE SYSTEM TO SET-UP */
    char *response;
    printf("Waiting for remote system . . . ");
    response = readLine(fd);
    printf("%s\n", response);
    free(response);

    /* WRITE USER INPUT TO SERIAL PORT */
    printf("Message: ");
    char ch = fgetc(stdin);
    char *message = malloc(sizeof(char) * BUFFER_SIZE);
    int index = 0;
    while (ch != '\n') {
        message[index] = ch;
        index++;
        ch = fgetc(stdin);
    }
    message[index++] = '\n';
    message[index] = '\0';
    int nbytes = (int) write(fd, message, index); // index is size of message
    if (nbytes < 0) {
        printf("Write to /dev/ttyACM0 failed.\n");
        return -1;
    }
    free(message);

    /* READ SERIAL PORT */
    response = readLine(fd);
    printf("Response: %s\n", response);
    free(response);

    /* CLOSE SERIAL PORT */
    close(fd);
    return 0;
}

char *readLine(int fd) {
    char ch = '\0';
    char *buffer = malloc(sizeof(char) * BUFFER_SIZE);
    int index = 0;
    int nbytes = 0;
    while ((nbytes = (int) read(fd, &ch, 1)) > 0) {
        buffer[index] = ch;
        index += nbytes;
        if (buffer[index-1] == '\n' || buffer[index-1] == '\r') {
            break;
        }
        buffer[index] = '\0';
    }
    return buffer;
}
