#ifndef HELPERS_H
#define HELPERS_H

#include "global.h"

int extract_cmd_msg(const char *buff, struct command_packet *packet);

char* build_data_str(struct transmission_packet *packet);

int compare_strings(const char *a, const char *b);

int time_format_check(const char *string);

#endif /* HELPERS_H */