#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include "includes.h"

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

    void sendErrorDataMsg(const char *format, ...);

    void sendInfoDataMsg(const char *format, ...);

    void sendTelemetry(char *buff);

};

#endif /* SERIAL_MANAGER_H */