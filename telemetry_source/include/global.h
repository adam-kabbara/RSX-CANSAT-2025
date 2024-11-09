#ifndef SET_VALS_H
#define SET_VALS_H

#include <Arduino.h>

#define XBEE_BAUD_RATE 57600
#define CMD_BUFF_SIZE 128
#define WORD_SIZE 64
#define DATA_SIZE 32
#define SENTENCE_SIZE 128

enum mode_control {
    SIM_ON = 1,
    SIM_OFF = 2,
    SIM_READY = 3
};

struct command_packet {
    char keyword[WORD_SIZE];
    char team_id[WORD_SIZE];
    char command[WORD_SIZE];
    char data[WORD_SIZE];
};

struct transmission_packet {
    int TEAM_ID;
    char MISSION_TIME;
    int PACKET_COUNT;
    char MODE[DATA_SIZE]; 
    char STATE[DATA_SIZE];
    float ALTITUDE;
    char TEMPERATURE[DATA_SIZE];
    char PRESSURE[DATA_SIZE];
    char VOLTAGE[DATA_SIZE];
    char GYRO_R[DATA_SIZE];
    char GYRO_P[DATA_SIZE];
    char GYRO_Y[DATA_SIZE];
    char ACCEL_R[DATA_SIZE];
    char ACCEL_P[DATA_SIZE];
    char ACCEL_Y[DATA_SIZE];
    char MAG_R[DATA_SIZE];
    char MAG_P[DATA_SIZE];
    char MAG_Y[DATA_SIZE];
    char AUTO_GYRO_ROTATION_RATE[DATA_SIZE];
    char GPS_TIME[DATA_SIZE];
    char GPS_ALTITUDE[DATA_SIZE];
    char GPS_LATITUDE[DATA_SIZE];
    char GPS_LONGITUDE[DATA_SIZE];
    char GPS_SATS[DATA_SIZE];
    char CMD_ECHO[DATA_SIZE];
};

#endif /* SET_VALS_H */