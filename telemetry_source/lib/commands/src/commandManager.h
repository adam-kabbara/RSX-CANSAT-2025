#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include "includes.h"
#include "serialManager.h"
#include "missionManager.h"
#include "sensorManager.h"
#include <LittleFS.h>

class CommandManager {
private:

    std::unordered_map<std::string, std::function<void(SerialManager&, MissionManager&, SensorManager&, const char*)>> command_map;

    // Command processing functions
    void do_cx(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_st(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_restart(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_give_status(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_sim(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_simp(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_cal(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_mec(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);
    void do_logs(SerialManager &ser, MissionManager &info, SensorManager &sensors, const char *data);

public:
    CommandManager();

    int processCommand(const char *cmd_buff, SerialManager &ser, MissionManager &info, SensorManager &sensors);
};

#endif /* COMMAND_MANAGER_H */