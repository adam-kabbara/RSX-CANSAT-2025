#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "includes.h"
#include "helpers.h"

class SerialManager
{
private:
    HardwareSerial* serialPort;

public:
    SerialManager(HardwareSerial& port) 
        : serialPort(&port) 
        {}

    void begin();

    int get_data(char *cmd_buff);

    void sendErrorMsg(const char *msg);

    void sendInfoMsg(const char *msg);

    void sendDataMsg(uint8_t err, const char *format, ...);

    void sendTelemetry(const transmission_packet *pckt);
};

#endif /* SERIAL_MANAGER_H */