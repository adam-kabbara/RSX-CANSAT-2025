#ifndef HELPERS_H
#define HELPERS_H

#include "includes.h"

const char* op_state_to_string(OperatingState state);

const char* op_mode_to_string(OperatingMode mode, int full);

void cmd_buff_to_echo(char *cmd_buff, char *cmd_echo);

void build_data_str(const transmission_packet *packet, char *buff, size_t size);

float pressure_to_alt(const float pressure);

#endif /* HELPERS_H */