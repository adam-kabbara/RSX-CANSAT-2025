#ifndef SET_VALS_H
#define SET_VALS_H

#include <Arduino.h>

#define XBEE_BAUD_RATE 57600
#define CMD_BUFF_SIZE 100
#define CMD_WORD_SIZE 50
#define GENERAL_WORD_SIZE 50

enum mode_control {
    FLIGHT = 0,
    SIM = 1,
    STANDBY = 2,
    SIM_READY = 3
};

struct command_packet {
    char keyword[CMD_WORD_SIZE];
    char team_id[CMD_WORD_SIZE];
    char command[CMD_WORD_SIZE];
    char data[CMD_WORD_SIZE];
};

#endif /* SET_VALS_H */