#include "serialManager.h"

void SerialManager::begin() 
{
    serialPort->begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(1000);
}

int SerialManager::get_data(char* cmd_buff)
{
    if(serialPort->available())
    {
        memset(cmd_buff, 0, CMD_BUFF_SIZE);
        byte bytes_read = serialPort->readBytesUntil('\n', cmd_buff, CMD_BUFF_SIZE - 1);
        cmd_buff[bytes_read] = '\0';
        return 1;
    }
    return 0;
}

void SerialManager::sendErrorMsg(const char* msg)
{
    char buffer[RESP_SIZE];
    snprintf(buffer, sizeof(buffer), "$E MSG:%s", msg);
    serialPort->println(buffer);
}

void SerialManager::sendInfoMsg(const char* msg)
{
    char buffer[RESP_SIZE];
    snprintf(buffer, sizeof(buffer), "$I MSG:%s", msg);
    serialPort->println(buffer);
}

void SerialManager::sendDataMsg(uint8_t err, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    char buffer[RESP_SIZE];

    vsnprintf(buffer, sizeof(buffer), format, args);
    
    if(err)
    {
        this->sendErrorMsg(buffer);
    }
    else
    {
        this->sendInfoMsg(buffer);
    }

    va_end(args);
}

void SerialManager::sendTelemetry(const transmission_packet *pckt)
{
    char buffer[DATA_BUFF_SIZE];
    build_data_str(pckt, buffer, DATA_BUFF_SIZE);
    serialPort->println(buffer);
}