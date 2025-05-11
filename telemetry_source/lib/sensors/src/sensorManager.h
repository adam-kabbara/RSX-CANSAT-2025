#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "includes.h"
#include "serialManager.h"
#include "missionManager.h"

class SensorManager
{
private:

    typedef struct transmission_packet {
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

    transmission_packet send_packet;

    typedef struct altitude_data {
        size_t window_size = ALTITUDE_WINDOW_SIZE;
        float buffer[ALTITUDE_WINDOW_SIZE];
        int idx = 0;
        float current_alt = 0.0;
        float max_alt = 0.0;
    } altitude_data;

    altitude_data alt_data;

public:

    OperatingState updateState(OperatingState curr_state);

    void sampleSensors(MissionManager &mission_info);

    void build_data_str(char *buff, size_t size);

    float pressure_to_alt(const float pressure);

    const char* op_mode_to_string(OperatingMode mode, int full);

    const char* op_state_to_string(OperatingState state);

    void cmd_buff_to_echo(char *cmd_buff);

    void setPacketCount(int count);

    void resetAltData();

    float getPressure();
};

#endif /* SENSOR_MANAGER_H */