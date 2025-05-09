#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H

#include "includes.h"
#include "serialManager.h"
#include <Preferences.h>
#include "helpers.h"

class CommandManager {
private:

    std::unordered_map<std::string, std::function<void(SerialManager&, mission_info_struct&, char*, Preferences preferences)>> command_map;

    // Command processing functions
    void do_cx(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_st(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_restart(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_give_status(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_sim(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_simp(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_cal(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);
    void do_mec(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences);

public:
    CommandManager();

    int processCommand(char *cmd_buff, SerialManager &ser, mission_info_struct &info, Preferences preferences);
};

#endif /* COMMAND_MANAGER_H */