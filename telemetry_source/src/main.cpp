#include "global.h"
#include "helpers.h"
#include <Preferences.h>

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
void do_give_status(const char *data);
/*------- FUNCTIONS ------*/

/*------- VARIABLES ------*/
hw_timer_t *send_timer = NULL;
int send_flag = 0;
char cmd_buff[CMD_BUFF_SIZE];
Preferences preferences;
int TRANSMISSION_ON = 0;
int SIMP_DATA = 0;
char *data_buff;
struct mission_info mission_info_live;
TaskHandle_t recv_main;
TaskHandle_t send_main;
/*------- VARIABLES ------*/

void setup() 
{

  // TODO: send info msg on processor reset

  // Set timer to transmit at 1Hz
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &send_1, true);
  timerAlarmWrite(send_timer, 1000000, true);

  // Clean flash memory
  // nvs_flash_erase(); // erase the NVS partition and...
  // nvs_flash_init(); // initialize the NVS partition.

  // Remove all preferences under the opened namespace
  //preferences.clear();

  // Retreive team id and mission time from flash memory
  preferences.begin("xb-set", true);
  mission_info_live.team_id = preferences.getInt("teamid", 1234);
  String mission_time_str = preferences.getString("missiontime", "00:00:00");
  mission_time_str.toCharArray(mission_info_live.mission_time, sizeof(mission_info_live.mission_time));
  preferences.end();

  Serial.begin(9600);
  delay(1000);

  Serial.println("\nSerial setup OK\n");

  // XBee Hardware serial
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

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
    while(TRANSMISSION_ON == 1)
    {
      // Fill transmission packet
      struct transmission_packet send_packet;

      // PRESSURE
      if(mission_info_live.mode_int == 1)
      {
        send_packet.PRESSURE = SIMP_DATA;
      }
      else // TODO: SENSOR DEPENDENT
      {
        send_packet.PRESSURE = random(0, 10000);;
      }
      float pressure_drop_rate = (last_pressure - send_packet.PRESSURE);
      last_pressure = send_packet.PRESSURE;

      // TEAM ID
      send_packet.TEAM_ID = mission_info_live.team_id;

      // MISSION TIME
      unsigned long current_time_ms = millis();
      unsigned long elapsed_time_s = current_time_ms - mission_info_live.mission_time_ms;
      unsigned long mission_time_hours = elapsed_time_s/3600;
      unsigned long mission_time_mins = (elapsed_time_s % 3600) / 60;
      unsigned long mission_time_s = elapsed_time_s % 60;
      char mission_time_str[9];
      sprintf(mission_time_str, "%02lu:%02lu:%02lu", mission_time_hours, mission_time_mins, mission_time_s);
      strncpy(send_packet.MISSION_TIME, mission_time_str, sizeof(send_packet.MISSION_TIME));
      send_packet.MISSION_TIME[sizeof(send_packet.MISSION_TIME) - 1] = '\0';
      preferences.begin("xb-set", false);
      preferences.putString("missiontime", mission_time_str);
      preferences.end();

      // MODE
      if(mission_info_live.mode_int == 0)
      {
        send_packet.MODE[0] = 'F'; // Store the character 'F'
        send_packet.MODE[1] = '\0'; // Null-terminate if you expect it to be treated as a string
      }
      else
      {
        send_packet.MODE[0] = 'S'; // Store the character 'F'
        send_packet.MODE[1] = '\0'; // Null-terminate if you expect it to be treated as a string
      }

      if(send_packet.PRESSURE >= 93000)
      {
        strncpy(send_packet.STATE, "LAUNCH_PAD", sizeof(send_packet.STATE));
        send_packet.STATE[sizeof(send_packet.STATE) - 1] = '\0'; // Ensure null-termination
      } else if (pressure_drop_rate > 50) {
        strncpy(send_packet.STATE, "ASCENT", sizeof(send_packet.STATE));
        send_packet.STATE[sizeof(send_packet.STATE) - 1] = '\0'; // Ensure null-termination
      } else if (pressure_drop_rate < -100 && send_packet.PRESSURE > 30000) {
        strncpy(send_packet.STATE, "DESCENT", sizeof(send_packet.STATE));
        send_packet.STATE[sizeof(send_packet.STATE) - 1] = '\0'; // Ensure null-termination
      } else if (pressure_drop_rate < -5 && send_packet.PRESSURE > 50000 && send_packet.PRESSURE < 90000) {
        strncpy(send_packet.STATE, "APOGEE", sizeof(send_packet.STATE));
        send_packet.STATE[sizeof(send_packet.STATE) - 1] = '\0'; // Ensure null-termination
      } else if (send_packet.PRESSURE > 70000 && send_packet.PRESSURE > 100) {
        strncpy(send_packet.STATE, "LANDED", sizeof(send_packet.STATE));
        send_packet.STATE[sizeof(send_packet.STATE) - 1] = '\0'; // Ensure null-termination
      }
      else {
        strncpy(send_packet.STATE, "IDLE", sizeof(send_packet.STATE));
        send_packet.STATE[sizeof(send_packet.STATE) - 1] = '\0'; // Ensure null-termination
      }
      // TODO: Probe release

      // ALTITUDE
      const float P0 = 101325.0;  // Sea level pressure in Pa
      const float T0 = 288.15;    // Sea level standard temperature in K
      const float L = 0.0065;     // Standard temperature lapse rate in K/m
      const float R = 287.05;     // Specific gas constant for dry air in J/(kg·K)
      const float g = 9.80665;    // Acceleration due to gravity in m/s²

      // Calculate altitude using the barometric formula
      // send_packet.ALTITUDE = (T0 / L) * (1 - pow((send_packet.PRESSURE / P0), (R * L) / (g)));
      send_packet.ALTITUDE = random(-10,11);

      // TODO: SENSOR DEPENDENT
      int ran_num[17];
      for(int i = 0; i < 17; i++)
      {
        ran_num[i] = random(-10, 11);
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
      strncpy(send_packet.GPS_TIME, send_packet.MISSION_TIME, sizeof(send_packet.GPS_TIME));
      send_packet.GPS_TIME[sizeof(send_packet.GPS_TIME) - 1] = '\0'; // Ensure null-termination
      send_packet.GPS_ALTITUDE = ran_num[13];
      send_packet.GPS_LATITUDE = ran_num[14];
      send_packet.GPS_LONGITUDE = ran_num[15];
      send_packet.GPS_SATS = ran_num[16];

      // CMD ECHO
      strncpy(send_packet.CMD_ECHO, mission_info_live.cmd_echo, sizeof(send_packet.CMD_ECHO));
      send_packet.CMD_ECHO[sizeof(send_packet.CMD_ECHO) - 1] = '\0'; // Ensure null-termination

      if(send_flag == 1)
      {
        mission_info_live.packet_count++;
        send_packet.PACKET_COUNT = mission_info_live.packet_count;
        data_buff = build_data_str(&send_packet);
        send_flag = 0;
        Serial2.println(data_buff);
        Serial.println("just send some data!");
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelay(pdMS_TO_TICKS(25));
  }
} // END: void loop()

void receive_commands(void *pvParameters)
{
  
  // Serial2.printf("Command reception started on core %d", xPortGetCoreID());

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
      
      if(compare_strings(recv_packet.keyword, "CMD") == 0)
      {
        safe_print("$IE MSG:REJECTED INPUT: NOT CMD");
        continue;
      }

      int team_id_chk_int;
      sscanf(recv_packet.team_id, "%d", &team_id_chk_int);
      if((team_id_chk_int != mission_info_live.team_id) && (compare_strings(recv_packet.command, "RESET_TEAM_ID") == 0))
      {
        char msg_size = snprintf(NULL, 0, "$IE MSG:REJECTED INPUT: TEAM ID DOES NOT MATCH. TEAM_ID is %d", mission_info_live.team_id) + 1;
        char message[msg_size];
        snprintf(message, msg_size, "$IE MSG:REJECTED INPUT: TEAM ID DOES NOT MATCH. TEAM_ID is %d", mission_info_live.team_id);
        safe_print(message);
        continue;
      }

      // Parse command
      int cmd_valid = 1;
      if(compare_strings(recv_packet.command, "CX")) do_cx(recv_packet.data);
      else if(compare_strings(recv_packet.command, "ST")) do_st(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIM")) do_sim(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIMP")) do_simp(recv_packet.data);
      else if(compare_strings(recv_packet.command, "CAL")) do_cal(recv_packet.data);
      else if(compare_strings(recv_packet.command, "MEC")) do_mec(recv_packet.data);
      else if(compare_strings(recv_packet.command, "RESET_TEAM_ID")) do_reset_team_id(recv_packet.data);
      else if(compare_strings(recv_packet.command, "TEST")) do_give_status(recv_packet.data);
      else
      {
        safe_print("$IE MSG:COMMAND REJECTED: NOT A COMMAND");
        cmd_valid = 0;
      }
      if(cmd_valid = 1)
      {
        strncpy(mission_info_live.cmd_echo, cmd_buff, sizeof(mission_info_live.cmd_echo));
        mission_info_live.cmd_echo[sizeof(mission_info_live.cmd_echo) - 1] = '\0'; // Ensure null-termination
      }
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }
} // END: receive_commands()

void do_give_status(const char *data)
{
  int msg_size = snprintf(NULL, 0, "$I MSG:TEST RECEIVED. CANSAT IS ONLINE.{%s|%s|%d}", mission_info_live.mode, mission_info_live.state, mission_info_live.team_id) + 1;
  char message[msg_size];
  snprintf(message, msg_size, "$I MSG:TEST RECEIVED. CANSAT IS ONLINE.{%s|%s|%d}", mission_info_live.mode, mission_info_live.state, mission_info_live.team_id);
  safe_print(message);
} // END: do_give_status

void do_cx(const char *data)
{
  if(compare_strings(data, "ON"))
  {
    if(TRANSMISSION_ON == 0)
    {
      safe_print("$I MSG:STARTING TELEMETRY TRANSMISSION");
      TRANSMISSION_ON = 1;
      timerWrite(send_timer, 0);
      timerAlarmEnable(send_timer);
    }
    else
    {
      safe_print("$IE MSG:TRANSMISSION IS ALREADY ON");
    }
  }
  else if(compare_strings(data, "OFF"))
  {
    if(TRANSMISSION_ON == 1)
    {
      timerAlarmDisable(send_timer);
      TRANSMISSION_ON = 0;
      safe_print("$I MSG:RECEIVED COMMAND: ENDING PAYLOAD TRANSMISSION");
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
  if(time_format_check(data) == 0)
  {
    mission_info_live.mission_time_ms = millis();
    int i = 0;
    while(data[i] != '\0')
    {
      mission_info_live.mission_time[i] = data[i];
      i++;
    }
    mission_info_live.mission_time[i] = '\0';

    char message[SENTENCE_SIZE];

    snprintf(message, SENTENCE_SIZE, "$I MSG:SET TIME TO: %s", mission_info_live.mission_time);
    safe_print(message);
  }
  else
  {
    safe_print("$IE MSG:DATA IS NOT VALID. SEND EITHER UTC TIME OR 'GPS'");
  }
} // END: do_st()

void do_sim(const char *data)
{
  if(TRANSMISSION_ON == 1)
  {
    safe_print("$IE MSG:SIMULATION MODE CANNOT BE CHANGED WHILE TRANSMISSION IS ON");
    return;
  }

  if(compare_strings(data, "ENABLE"))
  {
    switch(mission_info_live.sim_status)
    {
      case mission_info::OFF:
                  mission_info_live.sim_status = mission_info::ENABLED;
                  safe_print("$I MSG:SIMULATION MODE ENABLED");
                  break;
      case mission_info::ENABLED:
                  safe_print("$IE MSG:SIMULATION MODE IS ALREADY ENABLED");
                  break;
      case mission_info::ACTIVE:
                  safe_print("$IE MSG:SIMULATION MODE IS ALREADY ACTIVATED");
    }
  }
  else if(compare_strings(data, "ACTIVATE"))
  {
    switch(mission_info_live.sim_status)
    {
      case mission_info::ENABLED:
                  mission_info_live.sim_status = mission_info::ACTIVE;
                  mission_info_live.mode[0] = 'S';
                  mission_info_live.mode[1] = '\0';
                  mission_info_live.mode_int = 1;
                  safe_print("$I MSG:SET CANSAT TO SIMULATION MODE");
                  break;
      case mission_info::OFF:
                  safe_print("$IE MSG:SIMULATION MODE IS NOT ENABLED");
                  break;
      case mission_info::ACTIVE:
                  safe_print("$IE MSG:SIM MODE IS ALREADY ACTIVATED!");
    }
  }
  else if(compare_strings(data, "DISABLE"))
  {
    switch(mission_info_live.sim_status)
    {
      case mission_info::ACTIVE:
                  mission_info_live.sim_status = mission_info::OFF;
                  mission_info_live.mode[0] = 'F';
                  mission_info_live.mode[1] = '\0';
                  mission_info_live.mode_int = 0;
                  safe_print("$I MSG:CANSAT SET TO FLIGHT MODE");
                  break;
      case mission_info::ENABLED:
                  mission_info_live.sim_status = mission_info::OFF;
                  safe_print("$I  MSG:SIMULATION MODE DISABLED");
                  break;
      case mission_info::OFF:
                  safe_print("$IE MSG:SIMULATION MODE ALREADY DISABLED.");
                  break;
    }
  }
  else
  {
    int msg_size = snprintf(NULL, 0, "$IE MSG:UNRECOGNIZED SIM COMMAND: '%s'", data) + 1;
    char message[msg_size];
    snprintf(message, msg_size, "$IE MSG:UNRECOGNIZED SIM COMMAND: '%s'", data);
    safe_print(message);
  }
} // END: do_sim()

void do_simp(const char *data)
{
  // Check if we are in simulation mode
  if(mission_info_live.mode_int == 1)
  {
    SIMP_DATA = atoi(data);
  }
  else
  {
    safe_print("$IE MSG:CANNOT RECEIVE SIMP CMD, CANSAT IS IN FLIGHT MODE");
  }
} // END: do_simp()

void do_cal(const char *data)
{
  safe_print("$I MSG:RECEIVED COMMAND: CAL");

  // TODO: Calibrate altitude to 0 meters

  // TODO: Set up reset and processor recovery
} // END: do_Cal()

void do_mec(const char *data)
{
  safe_print("$I MSG:RECEIVED COMMAND: MEC");
  // TODO: complete after sensore are installed
} // END: do_mec()

void do_reset_team_id(const char *data)
{
  int new_team_id;
  int old_team_id = mission_info_live.team_id;
  sscanf(data, "%d", &new_team_id);
  
  // Write new team ID to flash memory
  preferences.begin("xb-set", false);
  preferences.putInt("teamid", new_team_id);
  mission_info_live.team_id = preferences.getInt("teamid", 1234);
  preferences.end();

  int msg_size = snprintf(NULL, 0, "$I MSG:TEAM ID CHANGED FROM %d TO %d", old_team_id, mission_info_live.team_id) + 1;
  char message[msg_size];
  snprintf(message, msg_size, "$I MSG:TEAM ID CHANGED FROM %d TO %d", old_team_id, mission_info_live.team_id);
  safe_print(message);
  
} // END: do_reset_team_id()

void safe_print(const char *msg)
{
  if(TRANSMISSION_ON == 0)
  {
    Serial2.println(msg);
  }
} // END: safe_print()

void IRAM_ATTR send_1()
{
  send_flag = 1;
} // END: send_1()
