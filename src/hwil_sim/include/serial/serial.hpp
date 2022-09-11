#pragma once

// C library headers
#include <string>

// Linux headers
#include <termios.h> // Contains POSIX terminal control definitions

class Serial
{
private:
    int m_serial_port;
    char m_read_buff [256];
    struct termios m_tty;

    bool connected = false;

public:
    Serial(std::string port, int baudrate);
    ~Serial();

    void closePort();
    void sendChar(unsigned char& msg);
    int readData(uint8_t *buffer, size_t length);

    bool isOpen() { return connected; }
};