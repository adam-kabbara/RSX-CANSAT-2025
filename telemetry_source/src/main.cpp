#include "global.h"
#include "helpers.h"
#include "esp_system.h"
#include <Preferences.h>
#include <stdarg.h>
#include <atomic>

/*------- DEFINITIONS ------*/
#define RX_PIN 16
#define TX_PIN 17
/*------- DEFINITIONS ------*/

/*------- FUNCTIONS ------*/
void send_data(void *pvParameters);
void receive_commands(void *pvParameters);
void IRAM_ATTR send_1();
void do_cx(const char *data);
void do_st(const char *data);
void do_sim(const char *data);
void do_simp(const char *data);
void do_cal(const char *data);
void do_mec(const char *data);
void do_reset_team_id(const char *data);
void safe_print(const char *msg);
void safe_print_data(const char *format, ...);
void do_give_status();
void do_restart();
/*------- FUNCTIONS ------*/

/*------- VARIABLES ------*/
hw_timer_t *send_timer = NULL;
int send_flag = 0;
char cmd_buff[CMD_BUFF_SIZE];
char cmd_echo[CMD_BUFF_SIZE];
char mission_time_str[DATA_SIZE];
Preferences preferences;
int SIMP_DATA = 0;
int ALT_CAL_CHK = 0;
char *data_buff;
struct mission_info_struct mission_info;
TaskHandle_t recv_main;
TaskHandle_t send_main;
/*------- VARIABLES ------*/

void setup() 
{
  // Set timer to transmit at 1Hz
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &send_1, true);
  timerAlarmWrite(send_timer, 1000000, true);

  // Debug serial
  //Serial.begin(9600);
  //delay(1000);

  // XBee Hardware serial
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  // Clean flash memory
  // nvs_flash_erase(); // erase the NVS partition
  // nvs_flash_init(); // initialize the NVS partition

  // Remove all preferences under the opened namespace
  // preferences.clear();

  esp_reset_reason_t reset_reason = esp_reset_reason();

  Serial2.println("$IE MSG:Processor restarted");
  switch (reset_reason) {
    case ESP_RST_SW:
      Serial2.println("$IE MSG:Reset Reason: Manual software trigger");
      break;
    case ESP_RST_UNKNOWN:
      Serial2.println("$IE MSG:Reset Reason: Unknown");
      break;
    case ESP_RST_POWERON:
      Serial2.println("$IE MSG:Reset Reason: Power On Reset");
      break;
    case ESP_RST_EXT:
      Serial2.println("$IE MSG:Reset Reason: External Reset");
      break;
    case ESP_RST_PANIC:
      Serial2.println("$IE MSG:Reset Reason: Software Reset due to Panic/Exception");
      break;
    case ESP_RST_INT_WDT:
      Serial2.println("$IE MSG:Reset Reason: Interrupt Watchdog Reset");
      break;
    case ESP_RST_TASK_WDT:
      Serial2.println("$IE MSG:Reset Reason: Task Watchdog Reset");
      break;
    case ESP_RST_WDT:
      Serial2.println("$IE MSG:Reset Reason: General Watchdog Reset");
      break;
    case ESP_RST_DEEPSLEEP:
      Serial2.println("$IE MSG:Reset Reason: Deep Sleep Wakeup");
      break;
    case ESP_RST_BROWNOUT:
      Serial2.println("$IE MSG:Reset Reason: Brownout Reset");
      break;
    case ESP_RST_SDIO:
      Serial2.println("$IE MSG:Reset Reason: SDIO Reset");
      break;
    default:
      Serial2.println("$IE MSG:Reset Reason: Unknown");
      break;
  }

  preferences.begin("xb-set", true);
  mission_info.team_id = preferences.getInt("teamid", 3114);
  mission_info.op_state = static_cast<mission_info_struct::OperatingState>(preferences.getInt("opstate", 6));

  if(mission_info.op_state != mission_info_struct::OperatingState::IDLE) 
  {
    Serial2.println("$IE MSG:Perfoming recovery as processor was not in IDLE state!");
    Serial2.println("$IE MSG:Reading previous mission time, launch altitude, packet count, mode, and state!");
    Serial2.println("$IE MSG:Data telemetry should resume momentarily!");
    // Get mission time, packet count, launch altitude
    String mission_time_s = preferences.getString("missiontime", "00:00:00");
    mission_time_s.toCharArray(mission_time_str, sizeof(mission_time_str));
    mission_info.launch_altitude = preferences.getFloat("grndalt", 0.0);
    mission_info.sim_status = static_cast<mission_info_struct::SimModeStatus>(preferences.getInt("simst", 0));
    mission_info.op_mode = static_cast<mission_info_struct::OperatingMode>(preferences.getInt("opmode", 0));
    ALT_CAL_CHK = 1;

    mission_info.packet_count = preferences.getInt("pckts", 0);
  }

  preferences.end();

  //Serial.println("Setup done! Starting Tasks...");
  Serial2.println("$I MSG:Setup done!");
  Serial2.println("$I MSG:Sending status");
  do_give_status();

  xTaskCreatePinnedToCore(
    receive_commands,      // Function to be performed when the task is called 
    "Receive Commands",    // Name of the task
    10000,         // Stack size
    NULL,         // Pointer that will be used as the parameter for the task being created
    2,           // Task Priority 
    &recv_main, // The task handler
    0          //xCoreID (Core 0)
  );

  delay(500);

  xTaskCreatePinnedToCore(
    send_data,      
    "Send Data",  
    10000,         
    NULL,         
    2,          
    &send_main, 
    1       
  );

} // END: void setup()

void loop()
{
  vTaskDelay(100);
}

// This function CANNOT write to ANY global parameters
void send_data(void *pvParameters)
{

  int last_pressure = 0;
  // Serial2.printf("Data transmission started on core %d", xPortGetCoreID());
  
  while(1)
  {
    // Wait for the transmission to be on
    while(mission_info.op_state != mission_info_struct::OperatingState::IDLE)
    {
      // Fill transmission packet
      struct transmission_packet send_packet;

      // ALTITUDE
      send_packet.ALTITUDE = random(-10,11);

      // TODO: Change state based on altitude
      strcpy(send_packet.STATE, op_state_to_string(mission_info.op_state));
      // TODO: Probe release

      // PRESSURE
      if(mission_info.op_mode == mission_info_struct::OperatingMode::SIM)
      {
        send_packet.PRESSURE = SIMP_DATA;
      }
      else // TODO: SENSOR DEPENDENT
      {
        send_packet.PRESSURE = random(0, 10000);
      }
      float pressure_drop_rate = (last_pressure - send_packet.PRESSURE);
      last_pressure = send_packet.PRESSURE;

      // TEAM ID
      send_packet.TEAM_ID = mission_info.team_id;

      // MISSION TIME
      time_t t;
      struct tm *now;

      time(&t);
      now = gmtime(&t);

      snprintf(mission_time_str, sizeof(mission_time_str), "%02d:%02d:%02d", now->tm_hour, now->tm_min, now->tm_sec);
      
      strcpy(send_packet.MISSION_TIME, mission_time_str);

      preferences.begin("xb-set", false);
      preferences.putString("missiontime", mission_time_str);
      preferences.end();

      // MODE
      strcpy(send_packet.MODE, op_mode_to_string(mission_info.op_mode, 0));

      // TODO: SENSOR DEPENDENT
      int ran_num[17];
      for(int i = 0; i < 17; i++)
      {
        ran_num[i] = random(0, 20);
      }

      send_packet.TEMPERATURE = ran_num[0];
      send_packet.VOLTAGE = ran_num[1];
      send_packet.GYRO_R = ran_num[2];
      send_packet.GYRO_P = ran_num[3];
      send_packet.GYRO_Y = ran_num[4];
      send_packet.ACCEL_R = ran_num[5];
      send_packet.ACCEL_P = ran_num[6];
      send_packet.ACCEL_Y = ran_num[7];
      send_packet.MAG_R = ran_num[8];
      send_packet.MAG_P = ran_num[9];
      send_packet.MAG_Y = ran_num[10];
      send_packet.AUTO_GYRO_ROTATION_RATE = ran_num[11];
      strcpy(send_packet.GPS_TIME, send_packet.MISSION_TIME);
      send_packet.GPS_ALTITUDE = ran_num[13];
      send_packet.GPS_LATITUDE = ran_num[14];
      send_packet.GPS_LONGITUDE = ran_num[15];
      send_packet.GPS_SATS = ran_num[16];

      // CMD ECHO
      strcpy(send_packet.CMD_ECHO, cmd_echo);

      if(send_flag == 1)
      {
        mission_info.packet_count++;
        send_packet.PACKET_COUNT = mission_info.packet_count;
        preferences.begin("xb-set", false);
        preferences.putInt("pckts", mission_info.packet_count);
        preferences.end();
        data_buff = build_data_str(&send_packet);
        send_flag = 0;
        Serial2.println(data_buff);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelay(pdMS_TO_TICKS(25));
  }
} // END: void loop()

void receive_commands(void *pvParameters)
{
  
  Serial.printf("Command reception started on core %d\n", xPortGetCoreID());

  // Receive commands from ground station
  while(1)
  {
    
    if(Serial2.available())
    {
      memset(cmd_buff, 0, CMD_BUFF_SIZE);
      byte bytes_read = Serial2.readBytesUntil('\n', cmd_buff, CMD_BUFF_SIZE - 1);
      cmd_buff[bytes_read] = '\0';

      if(bytes_read == CMD_BUFF_SIZE - 1)
      {
        safe_print("$IE MSG:COMMAND REJECTED: LONGER THAN ALLOCATED BUFFER SIZE");
        continue;
      }

      // Extract data
      struct command_packet recv_packet;
      
      if(extract_cmd_msg(cmd_buff, &recv_packet) > 0)
      {
        safe_print("$IE MSG:COMMAND REJECTED: FORMAT IS INCORRECT. EXPECTING 'CMD,<TEAM_ID>,<CMD>,<DATA>'");
        continue;
      }
      
      if(compare_strings(recv_packet.keyword, "CMD") != 0)
      {
        safe_print("$IE MSG:REJECTED INPUT: NOT CMD");
        continue;
      }

      int team_id_chk_int;
      sscanf(recv_packet.team_id, "%d", &team_id_chk_int);
      if((team_id_chk_int != mission_info.team_id) && (compare_strings(recv_packet.command, "RESET_TEAM_ID") != 0))
      {
        safe_print_data("$IE MSG:REJECTED INPUT: TEAM ID DOES NOT MATCH. TEAM_ID is %d", mission_info.team_id);
        continue;
      }

      // Parse command
      int cmd_valid = 1;
      if(compare_strings(recv_packet.command, "CX") == 0) do_cx(recv_packet.data);
      else if(compare_strings(recv_packet.command, "ST") == 0) do_st(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIM") == 0) do_sim(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIMP") == 0) do_simp(recv_packet.data);
      else if(compare_strings(recv_packet.command, "CAL") == 0) do_cal(recv_packet.data);
      else if(compare_strings(recv_packet.command, "MEC") == 0) do_mec(recv_packet.data);
      else if(compare_strings(recv_packet.command, "RR") == 0) do_restart();
      else if(compare_strings(recv_packet.command, "TMID") == 0) do_reset_team_id(recv_packet.data);
      else if(compare_strings(recv_packet.command, "TEST") == 0) do_give_status();
      else
      {
        safe_print("$IE MSG:COMMAND REJECTED: NOT A COMMAND");
        cmd_valid = 0;
      }
      if(cmd_valid == 1)
      {
        cmd_buff_to_echo(cmd_buff, cmd_echo);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }
} // END: receive_commands()

void do_give_status()
{
  safe_print_data("$I MSG:CANSAT IS ONLINE.{%s|%s}", 
      op_mode_to_string(mission_info.op_mode, 1), op_state_to_string(mission_info.op_state));
} // END: do_give_status

void do_restart()
{
  safe_print("$I MSG:Attempting to restart processor!");
  ESP.restart();
} // END: do_restart

void do_cx(const char *data)
{
  if(compare_strings(data, "ON") == 0)
  {
    if(mission_info.op_state == mission_info_struct::OperatingState::IDLE && (ALT_CAL_CHK == 1 || mission_info.op_mode == mission_info_struct::OperatingMode::SIM))
    {
      mission_info.packet_count = 0;
      safe_print("$I MSG:STARTING TELEMETRY TRANSMISSION");
      mission_info.op_state = mission_info_struct::OperatingState::LAUNCH_PAD;
      timerWrite(send_timer, 0);
      timerAlarmEnable(send_timer);
    }
    else if(mission_info.op_state != mission_info_struct::OperatingState::IDLE)
    {
      safe_print("$IE MSG:TRANSMISSION IS ALREADY ON");
    }
    else
    {
      safe_print("$IE MSG: CANNOT START TELEMETRY BEFORE CALIBRATING ALTITUDE!");
    }
  }
  else if(compare_strings(data, "OFF") == 0)
  {
    if(mission_info.op_state != mission_info_struct::OperatingState::IDLE)
    {
      timerAlarmDisable(send_timer);
      mission_info.op_state = mission_info_struct::OperatingState::IDLE;
      safe_print_data("$I MSG:ENDING PAYLOAD TRANSMISSION.{%s|%s}", 
        op_mode_to_string(mission_info.op_mode, 1), op_state_to_string(mission_info.op_state));
    }
    else
    {
      safe_print("$IE MSG:TRANSMISSION IS ALREADY OFF");
    }
  }
  else
  {
    safe_print("$IE MSG:DATA IS NOT VALID; SEND ON/OFF");
  }
} // END: do_cx()

void do_st(const char *data)
{
  if(compare_strings(data, "GPS") == 0)
  {
    safe_print("$IE MSG:GPS IS CURRENTLY NOT SUPPORTED!");
  }
  else if(set_time_from_str(data) == 0)
  {
    time_t t;
    struct tm *utc_time;

    time(&t);
    utc_time = gmtime(&t);

    safe_print_data("$I MSG:TIME SET! CURRENT TIME: %02d:%02d:%02d", utc_time->tm_hour, utc_time->tm_min, utc_time->tm_sec);
  }
  else
  {
    safe_print("$IE MSG:FAILED TO SET TIME");
  }
} // END: do_st()

void do_sim(const char *data)
{
  if(mission_info.op_state != mission_info_struct::OperatingState::IDLE)
  {
    safe_print("$IE MSG:SIMULATION MODE CANNOT BE CHANGED WHILE TRANSMISSION IS ON");
    return;
  }

  if(compare_strings(data, "ENABLE") == 0)
  {
    switch(mission_info.sim_status)
    {
      case mission_info_struct::SimModeStatus::OFF:
        {
          mission_info.sim_status = mission_info_struct::SimModeStatus::ENABLED;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(mission_info.sim_status);
          preferences.putInt("simst", sim_status_int);
          preferences.end();
          safe_print("$I MSG:SIMULATION MODE ENABLED");
          break;
        }
      case mission_info_struct::SimModeStatus::ENABLED:
        {
          safe_print("$IE MSG:SIMULATION MODE IS ALREADY ENABLED");
          break;
        }
      case mission_info_struct::SimModeStatus::ACTIVE:
        {
          safe_print("$IE MSG:SIMULATION MODE IS ALREADY ACTIVATED");
          break;
        }
    }
  }
  else if(compare_strings(data, "ACTIVATE") == 0)
  {
    switch(mission_info.sim_status)
    {
      case mission_info_struct::SimModeStatus::ENABLED:
        {
          mission_info.sim_status = mission_info_struct::SimModeStatus::ACTIVE;
          mission_info.op_mode = mission_info_struct::OperatingMode::SIM;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(mission_info.sim_status);
          preferences.putInt("simst", sim_status_int);
          int op_mode_int = static_cast<int>(mission_info.op_mode);
          preferences.putInt("opmode", op_mode_int);
          preferences.end();
          safe_print_data("$I MSG:SET CANSAT TO SIMULATION MODE.{%s|%s}", 
            op_mode_to_string(mission_info.op_mode, 1), op_state_to_string(mission_info.op_state));
          break;
        }
      case mission_info_struct::SimModeStatus::OFF:
        {
          safe_print("$IE MSG:SIMULATION MODE IS NOT ENABLED");
          break;
        }
      case mission_info_struct::SimModeStatus::ACTIVE:
        {
          safe_print("$IE MSG:SIM MODE IS ALREADY ACTIVATED!");
          break;
        }
    }
  }
  else if(compare_strings(data, "DISABLE") == 0)
  {
    switch(mission_info.sim_status)
    {
      case mission_info_struct::SimModeStatus::ACTIVE:
        {
          mission_info.sim_status = mission_info_struct::SimModeStatus::OFF;
          mission_info.op_mode = mission_info_struct::OperatingMode::FLIGHT;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(mission_info.sim_status);
          preferences.putInt("simst", sim_status_int);
          int op_mode_int = static_cast<int>(mission_info.op_mode);
          preferences.putInt("opmode", op_mode_int);
          preferences.end();
          safe_print_data("$I MSG:SET CANSAT TO FLIGHT MODE.{%s|%s}", 
            op_mode_to_string(mission_info.op_mode, 1), op_state_to_string(mission_info.op_state));
          break;
        }
      case mission_info_struct::SimModeStatus::ENABLED:
        {
          mission_info.sim_status = mission_info_struct::SimModeStatus::OFF;
          preferences.begin("xb-set", false);
          int sim_status_int = static_cast<int>(mission_info.sim_status);
          preferences.putInt("simst", sim_status_int);
          preferences.end();
          safe_print("$I  MSG:SIMULATION MODE DISABLED");
          break;
        }
      case mission_info_struct::SimModeStatus::OFF:
        {
          safe_print("$IE MSG:SIMULATION MODE ALREADY DISABLED.");
          break;
        }
    }
  }
  else
  {
    safe_print_data("$IE MSG:UNRECOGNIZED SIM COMMAND: '%s'", data);
  }
} // END: do_sim()

void do_simp(const char *data)
{
  // Check if we are in simulation mode
  if(mission_info.op_mode == mission_info_struct::OperatingMode::SIM)
  {
    int i;
    sscanf(data, "%d", i);
    SIMP_DATA = i;
  }
  else
  {
    safe_print("$IE MSG:CANNOT RECEIVE SIMP CMD, CANSAT IS IN FLIGHT MODE");
  }
} // END: do_simp()

void do_cal(const char *data)
{
  // TODO add sensor function here
  mission_info.launch_altitude = 100;
  ALT_CAL_CHK = 1;
  preferences.begin("xb-set", false);
  preferences.putFloat("grndalt", mission_info.launch_altitude);
  preferences.end();
  safe_print_data("$I MSG:Launch Altitude calibrated to %f", mission_info.launch_altitude);
} // END: do_Cal()

void do_mec(const char *data)
{
  safe_print("$I MSG:RECEIVED COMMAND: MEC");
  // TODO: complete after sensore are installed
} // END: do_mec()

void do_reset_team_id(const char *data)
{
  int new_team_id;
  int old_team_id = mission_info.team_id;
  sscanf(data, "%d", &new_team_id);
  
  // Write new team ID to flash memory
  preferences.begin("xb-set", false);
  preferences.putInt("teamid", new_team_id);
  mission_info.team_id = preferences.getInt("teamid", 1234);
  preferences.end();

  safe_print_data("$I MSG:TEAM ID CHANGED FROM %d TO %d", old_team_id, mission_info.team_id);
  
} // END: do_reset_team_id()

void safe_print(const char *msg)
{
  if(mission_info.op_state == mission_info_struct::OperatingState::IDLE)
  {
    Serial2.println(msg);
  }
} // END: safe_print()

void safe_print_data(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  if(mission_info.op_state == mission_info_struct::OperatingState::IDLE)
  {
    char message[CMD_BUFF_SIZE];

    int msg_size = vsnprintf(message, sizeof(message), format, args);
    
    Serial2.println(message); 

    if(msg_size >= CMD_BUFF_SIZE)
    {
      Serial2.println("WARNING: CHECK FSW CODE, RET MSG IS TRUNCATED");
    }
  }

  va_end(args);
}

void IRAM_ATTR send_1()
{
  send_flag = 1;
} // END: send_1()
