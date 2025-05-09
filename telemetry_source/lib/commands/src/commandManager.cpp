#include "commandManager.h"

CommandManager::CommandManager() 
{
    // Initialize the map with corresponding functions
    using namespace std::placeholders;
    command_map.emplace("CX", std::bind(&CommandManager::do_cx, this, _1, _2, _3, _4));
    command_map.emplace("ST", std::bind(&CommandManager::do_st, this, _1, _2, _3, _4));
    command_map.emplace("RR", std::bind(&CommandManager::do_restart, this, _1, _2, _3, _4));
    command_map.emplace("TEST", std::bind(&CommandManager::do_give_status, this, _1, _2, _3, _4));
    command_map.emplace("SIM", std::bind(&CommandManager::do_sim, this, _1, _2, _3, _4));
    command_map.emplace("SIMP", std::bind(&CommandManager::do_simp, this, _1, _2, _3, _4));
    command_map.emplace("CAL", std::bind(&CommandManager::do_cal, this, _1, _2, _3, _4));
    command_map.emplace("MEC", std::bind(&CommandManager::do_mec, this, _1, _2, _3, _4));
}

int CommandManager::processCommand(char *cmd_buff, SerialManager &ser, mission_info_struct &info, Preferences preferences) 
{
    // Extract fields from command buffer

    struct command_packet 
    {
        char *keyword = nullptr;
        char *team_id = nullptr;
        char *command = nullptr;
        char *data = nullptr;
    };

    command_packet packet;

    char *token = strtok(cmd_buff, ",");
    int token_cnt = 0;

    while(token != NULL)
    {
        switch(token_cnt)
        {
            case 0: 
                packet.keyword = token; 
                break;
            case 1: 
                packet.team_id = token; 
                break;
            case 2: 
                packet.command = token; 
                break;
        }
        token_cnt++;

        if(token_cnt == 3)
        {
          token = strtok(NULL, "");
          packet.data = token;
        }
        else
        {
          token = strtok(NULL, ",");
        }
    }

    // Check validity
    if(!packet.keyword || !packet.team_id || !packet.command || !packet.data)
    {
        ser.sendErrorMsg("COMMAND REJECTED: FORMAT IS INCORRECT.");
        ser.sendDataMsg(1, "COMMAND: %s\n", cmd_buff);
        return 0;
    }

    if(strcmp(packet.keyword, "CMD") != 0)
    {
        ser.sendErrorMsg("COMMAND REJECTED: FIRST FIELD MUST BE 'CMD'.");
        return 0;
    }

    if(atoi(packet.team_id) != TEAM_ID)
    {
        ser.sendDataMsg(1, "COMMAND REJECTED: TEAM ID DOES NOT MATCH EXPECTED VALUE OF %d", TEAM_ID);
        return 0;
    }

    // Check if the command is in the map and call the corresponding function
    auto iter = command_map.find(packet.command);
    if (iter == command_map.end())
    {
        ser.sendErrorMsg("COMMAND REJECTED: NOT A COMMAND");
        return 0;
    }
    iter->second(ser, info, packet.data, preferences);
    return 1;
}

// Toggle mission telemetry
void CommandManager::do_cx(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
    if(strcmp(data, "ON") == 0)
    {
        if(info.op_state == IDLE && info.ALT_CAL_CHK == 1)
        {
            info.packet_count = 0;
            ser.sendInfoMsg("STARTING TELEMETRY TRANSMISSION.");
            info.op_state = LAUNCH_PAD;
        }
        else if(info.op_state != IDLE)
        {
            ser.sendErrorMsg("TRANSMISSION IS ALREADY ON.");
        }
        else
        {
            ser.sendErrorMsg("CANNOT START TELEMETRY BEFORE CALIBRATING ALTITUDE!");
        }
    }
    else if(strcmp(data, "OFF") == 0)
    {
        if(info.op_state != IDLE)
        {
            info.op_state = IDLE;
            info.ALT_CAL_CHK = 0;
            ser.sendDataMsg(0, "ENDING PAYLOAD TRANSMISSION.{%s|%s}", 
                op_mode_to_string(info.op_mode, 1), op_state_to_string(info.op_state));
        }
        else
        {
            ser.sendErrorMsg("TRANSMISSION IS ALREADY OFF!");
        }
    }
    else
    {
        ser.sendErrorMsg("DATA IS NOT VALID; SEND ON/OFF");
    }
}

void CommandManager::do_st(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
    if(strcmp(data, "GPS") == 0)
    {
        ser.sendInfoMsg("TODO: GPS!");
    }
    else if(std::strstr(data, "UTC") != nullptr)
    {
        ser.sendInfoMsg("TODO: SET RTC CONTROLLER TIME!");
    }
    else
    {
        ser.sendErrorMsg("DATA IS NOT VALID; SEND 'GPS' OR UTC TIME");
    }
}

void CommandManager::do_give_status(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
  ser.sendDataMsg(0, "CANSAT IS ONLINE.{%s|%s}", 
      op_mode_to_string(info.op_mode, 1), op_state_to_string(info.op_state));
} // END: do_give_status

void CommandManager::do_restart(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
  ser.sendInfoMsg("Attempting to restart processor!");
  ESP.restart();
} // END: do_restart

void CommandManager::do_sim(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
  if(info.op_state != IDLE)
  {
    ser.sendErrorMsg("SIMULATION MODE CANNOT BE CHANGED WHILE TRANSMISSION IS ON");
    return;
  }

  if(strcmp(data, "ENABLE") == 0)
  {
    switch(info.sim_status)
    {
      case SIM_OFF:
        {
          info.sim_status = SIM_EN;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(info.sim_status);
          preferences.putInt("simst", sim_status_int);
          preferences.end();
          ser.sendInfoMsg("SIMULATION MODE ENABLED");
          break;
        }
      case SIM_EN:
        {
          ser.sendErrorMsg("SIMULATION MODE IS ALREADY ENABLED");
          break;
        }
      case SIM_ON:
        {
          ser.sendErrorMsg("SIMULATION MODE IS ALREADY ACTIVATED");
          break;
        }
    }
  }
  else if(strcmp(data, "ACTIVATE") == 0)
  {
    switch(info.sim_status)
    {
      case SIM_EN:
        {
          info.sim_status = SIM_ON;
          info.op_mode = OPMODE_SIM;
          info.FIRST_SIMP = 0;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(info.sim_status);
          preferences.putInt("simst", sim_status_int);
          int op_mode_int = static_cast<int>(info.op_mode);
          preferences.putInt("opmode", op_mode_int);
          preferences.end();
          ser.sendDataMsg(0, "SIMULATION MODE IS ACTIVE{%s|%s}", 
            op_mode_to_string(info.op_mode, 1), op_state_to_string(info.op_state));
          info.ALT_CAL_CHK = 1;
          break;
        }
      case SIM_OFF:
        {
          ser.sendErrorMsg("SIMULATION MODE IS NOT ENABLED");
          break;
        }
      case SIM_ON:
        {
          ser.sendErrorMsg("SIMULATION MODE IS ALREADY ACTIVATED");
          break;
        }
    }
  }
  else if(strcmp(data, "DISABLE") == 0)
  {
    switch(info.sim_status)
    {
      case SIM_ON:
        {
          info.sim_status = SIM_OFF;
          info.op_mode = OPMODE_FLIGHT;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(info.sim_status);
          preferences.putInt("simst", sim_status_int);
          int op_mode_int = static_cast<int>(info.op_mode);
          preferences.putInt("opmode", op_mode_int);
          preferences.end();
          ser.sendDataMsg(0, "SET CANSAT TO FLIGHT MODE.{%s|%s}", 
            op_mode_to_string(info.op_mode, 1), op_state_to_string(info.op_state));
          break;
        }
      case SIM_EN:
        {
          info.sim_status = SIM_OFF;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(info.sim_status);
          preferences.putInt("simst", sim_status_int);
          preferences.end();
          ser.sendErrorMsg("SIMULATION MODE DISABLED");
          break;
        }
      case SIM_OFF:
        {
          ser.sendErrorMsg("SIMULATION MODE ALREADY DISABLED.");
          break;
        }
    }
  }
  else
  {
    ser.sendDataMsg(1, "UNRECOGNIZED SIM COMMAND: '%s'", data);
  }
} // END: do_sim()

void CommandManager::do_simp(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
  // Check if we are in simulation mode
  if(info.op_mode == OPMODE_SIM)
  {
    int i;
    sscanf(data, "%d", i);
    if(info.FIRST_SIMP == 0)
    {
        info.launch_altitude = pressure_to_alt(i/100.0);
        info.FIRST_SIMP = 1;
    }
    info.SIMP_DATA = i;
  }
  else
  {
    ser.sendErrorMsg("CANNOT RECEIVE SIMP CMD, CANSAT IS IN FLIGHT MODE");
  }
} // END: do_simp()

void CommandManager::do_cal(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
  // TODO add sensor function here
  info.launch_altitude = pressure_to_alt(900.0);
  info.ALT_CAL_CHK = 1;
  preferences.begin("xb-set", false);
  preferences.putFloat("grndalt", info.launch_altitude);
  preferences.end();
  ser.sendDataMsg(0, "Launch Altitude calibrated to %f", info.launch_altitude);
} // END: do_Cal()

void CommandManager::do_mec(SerialManager &ser, mission_info_struct &info, const char *data, Preferences preferences)
{
  ser.sendInfoMsg("MEC COMMAND RECVD");
}