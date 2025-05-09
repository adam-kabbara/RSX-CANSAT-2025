#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "includes.h"

class SensorManager
{
private:

typedef struct transmission_packet{
    int TEAM_ID_PCKT;
    char MISSION_TIME[DATA_SIZE];
    int PACKET_COUNT;
    char MODE[2]; 
    char STATE[DATA_SIZE];
    float ALTITUDE;
    float TEMPERATURE;
    float PRESSURE;
    float VOLTAGE;
    int GYRO_R;
    int GYRO_P;
    int GYRO_Y;
    int ACCEL_R;
    int ACCEL_P;
    int ACCEL_Y;
    int MAG_R;
    int MAG_P;
    int MAG_Y;
    int AUTO_GYRO_ROTATION_RATE;
    char GPS_TIME[DATA_SIZE];
    float GPS_ALTITUDE;
    float GPS_LATITUDE;
    float GPS_LONGITUDE;
    int GPS_SATS;
    char CMD_ECHO[CMD_BUFF_SIZE];
} transmission_packet;

transmission_packet data_packet;

public:


};

#endif /* SENSOR_MANAGER_H */