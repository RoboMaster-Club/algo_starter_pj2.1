// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

typedef struct _Package {
    float yaw;
    float pitch;
} Package;

int main(int argc, char **argv)
{
    int serial_port = open("/dev/ttyTHS0", O_RDWR);

    // Check for errors
    if (serial_port < 0)
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8; // 8 bits per byte (most common)

    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)


    tty.c_lflag &= ~ICANON; // Disable canonical mode

    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo

    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP


    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes


    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)

    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Block read for 1 second max
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 10;

    cfsetispeed(&tty, B1152000);
    cfsetospeed(&tty, B1152000);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // unsigned char msg[] = {'U', 'U', 'U', 'U', 'U', 'U'};
    Package pkout = {-1.f, 1.f};
    write(serial_port, &pkout, sizeof(pkout));

    fsync(serial_port);
    
    unsigned char buffer[sizeof(Package)];
    memset(&buffer, '\0', sizeof(buffer));

    int num_bytes = read(serial_port, &buffer, sizeof(buffer));

    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
        return 1;
    }

    Package packageIn = *(Package*)buffer;
    // printf("Read %i bytes. Received message: %s", num_bytes, buffer);

    printf("%f %f\n", packageIn.yaw, packageIn.pitch);
    close(serial_port);


    return 0;
}