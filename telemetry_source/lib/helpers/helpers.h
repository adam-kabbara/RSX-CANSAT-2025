#ifndef HELPERS_H
#define HELPERS_H

#include "global.h"

int extract_cmd_msg(const char *buff, struct command_packet *packet);

int comapre_strings(const char *a, const char *b);

#endif /* HELPERS_H */