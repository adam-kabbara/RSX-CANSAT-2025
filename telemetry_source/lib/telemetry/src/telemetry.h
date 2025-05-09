#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "includes.h"
#include "serialManager.h"
#include "commandManager.h"
#include "helpers.h"
#include <Preferences.h>

void resetSeq(SerialManager &serial, mission_info_struct &mission_info, Preferences preferences);

OperatingState updateState(OperatingState curr_state, float *alt_buff, size_t size, int idx, float &max_alt);

void sampleSensors(mission_info_struct &mission_info, transmission_packet &send_packet, float *alt_buff, size_t size, int &idx, float &max_alt);

#endif /* TELEMETRY_H */