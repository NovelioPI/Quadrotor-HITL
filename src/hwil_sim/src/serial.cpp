#include <serial/serial.hpp>
#include <unistd.h> // write(), read(), close()
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <stdio.h>
#include <string.h>

void Serial::closePort()
{
    close(m_serial_port);
    printf("Terminate object\n");
}

void Serial::sendChar(unsigned char& msg)
{
    try
    {
        unsigned char cmd [] = {msg};
        if (m_serial_port < 0) {printf("No board connected\n");throw 1;}
        (void) write(m_serial_port, cmd, sizeof(cmd));
    }
    catch(int err)
    {
        printf("Cannot send serial data!\n");
    }
    
}

int Serial::readData(uint8_t *buffer, size_t length)
{
    memset(buffer, '\0', sizeof(buffer));
    int num_bytes = read(m_serial_port, buffer, length);

    try
    {
        if (m_serial_port < 0) 
        {
            printf("Error reading: %s\n", strerror(errno));
            throw 1;
        }

        return num_bytes;
    }

    catch(int err)
    {
        printf("No data went through!\n");
        return 0;
    }

}

Serial::Serial(std::string port, int baudrate)
{
    m_serial_port = open(port.c_str(), O_RDWR);
    try
    {
        if (m_serial_port < 0) {printf("Error %i from open: %s\n", errno, strerror(errno));throw 1;}
        else{printf("Successfully opened the port!\n");}

        if(tcgetattr(m_serial_port, &m_tty) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));throw 1;
        }

        m_tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        m_tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        m_tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
        m_tty.c_cflag |= CS8; // 8 bits per byte (most common)
        m_tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        m_tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        m_tty.c_lflag &= ~ICANON;
        m_tty.c_lflag &= ~ECHO; // Disable echo
        m_tty.c_lflag &= ~ECHOE; // Disable erasure
        m_tty.c_lflag &= ~ECHONL; // Disable new-line echo
        m_tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        m_tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        m_tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        m_tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        m_tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // m_tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // m_tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
        m_tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        m_tty.c_cc[VMIN] = 0;

        cfsetispeed(&m_tty, baudrate);
        cfsetospeed(&m_tty, baudrate);

        if (tcsetattr(m_serial_port, TCSANOW, &m_tty) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));throw 1;
        }

        connected = true;
    }

    catch(int err)
    {
        printf("No board connected!\n");
    }
}

Serial::~Serial()
{
    closePort();
}