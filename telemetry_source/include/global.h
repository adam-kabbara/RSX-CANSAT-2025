#ifndef SET_VALS_H
#define SET_VALS_H

#include <Arduino.h>
#include <time.h>

#define XBEE_BAUD_RATE 57600
#define CMD_BUFF_SIZE 128
#define WORD_SIZE 64
#define DATA_SIZE 32
#define SENTENCE_SIZE 128
#define DATA_BUFF_SIZE 528

struct mission_info_struct {
    enum SimModeStatus {
        OFF = 0,
        ENABLED = 1,
        ACTIVE = 2
    };
    enum OperatingMode {
        FLIGHT = 0,
        SIM = 1
    };
    enum OperatingState {
        LAUNCH_PAD = 0,
        ASCENT = 1,
        APOGEE = 2,
        DESCENT = 3,
        PROBE_RELEASE = 4,
        LANDED = 5,
        IDLE = 6
    };
    OperatingState op_state = IDLE;
    SimModeStatus sim_status = OFF;
    OperatingMode op_mode = FLIGHT;
    int team_id = 3114;
    int packet_count = 100;
    float launch_altitude = 0.0;
};

struct command_packet {
    char keyword[WORD_SIZE];
    char team_id[WORD_SIZE];
    char command[WORD_SIZE];
    char data[WORD_SIZE];
};

struct transmission_packet {
    int TEAM_ID;
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
};

#endif /* SET_VALS_H */