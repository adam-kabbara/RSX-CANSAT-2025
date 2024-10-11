#ifndef SET_VALS_H
#define SET_VALS_H

#define XBEE_BAUD_RATE 57600
#define CMD_BUFF_SIZE 100
#define CMD_WORD_SIZE 50

enum mode_control {
    FLIGHT = 0,
    SIM = 1,
    STANDBY = 2,
    SIM_READY = 3
};

struct command_packet {
    char[CMD_WORD_SIZE] keyword;
    char[CMD_WORD_SIZE] team_id_check;
    char[CMD_WORD_SIZE] commad;
    char[CMD_WORD_SIZE] data;
};

#endif /* SET_VALS_H */