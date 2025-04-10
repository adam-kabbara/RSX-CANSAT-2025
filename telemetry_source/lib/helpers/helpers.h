#ifndef HELPERS_H
#define HELPERS_H

#include "global.h"

int extract_cmd_msg(const char *buff, struct command_packet *packet);

const char* op_state_to_string(mission_info_struct::OperatingState state);

const char* op_mode_to_string(mission_info_struct::OperatingMode mode, int full);

void cmd_buff_to_echo(char *cmd_buff, char *cmd_echo);

char* build_data_str(struct transmission_packet *packet);

int compare_strings(const char *a, const char *b);

int set_time_from_str(const char *time_utc);

#endif /* HELPERS_H */