#ifndef SET_VALS_H
#define SET_VALS_H

#include <Arduino.h>

#define XBEE_BAUD_RATE 57600
#define CMD_BUFF_SIZE 128
#define WORD_SIZE 64
#define DATA_SIZE 32
#define SENTENCE_SIZE 128
#define DATA_BUFF_SIZE 256

struct mission_info {
    enum Sim_Mode_Status {
        OFF = 0,
        ENABLED = 1,
        ACTIVE = 2
    };
    char mode[WORD_SIZE] = "F";
    int mode_int = 0; // F = 0, S = 1
    char state[WORD_SIZE] = "UNKNOWN";
    Sim_Mode_Status sim_status = OFF;
    int team_id = 0;
    char mission_time[WORD_SIZE] = "00:00:00";
    unsigned long mission_time_ms = 0;
    int packet_count = 0;
    char cmd_echo[WORD_SIZE] = "NONE";
};

struct command_packet {
    char keyword[WORD_SIZE];
    char team_id[WORD_SIZE];
    char command[WORD_SIZE];
    char data[WORD_SIZE];
};

struct transmission_packet {
    int TEAM_ID;
    char MISSION_TIME[9];
    int PACKET_COUNT;
    char MODE[1]; 
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
    char GPS_TIME[9];
    float GPS_ALTITUDE;
    float GPS_LATITUDE;
    float GPS_LONGITUDE;
    int GPS_SATS;
    char CMD_ECHO[WORD_SIZE];
};

#endif /* SET_VALS_H */