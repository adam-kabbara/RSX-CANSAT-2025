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
/*------- FUNCTIONS ------*/

/*------- VARIABLES ------*/
hw_timer_t *send_timer = NULL;
char cmd_buff[CMD_BUFF_SIZE];
Preferences preferences;
int TEAM_ID;
char MISSION_TIME[WORD_SIZE] = "00:00:00";
mode_control current_mode = SIM_OFF;
int TRANSMISSION_ON = 0;
int SEND_FLAG = 0;
struct transmission_packet send_packet;
TaskHandle_t recv_main;
TaskHandle_t send_main;
/*------- VARIABLES ------*/

void setup() 
{

  // Set timer to transmit at 1Hz
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &send_1, true);
  timerAlarmWrite(send_timer, 1000000, true);

  // Clean flash memory
  // nvs_flash_erase(); // erase the NVS partition and...
  // nvs_flash_init(); // initialize the NVS partition.

  // Remove all preferences under the opened namespace
  //preferences.clear();

  // Retreive team id from flash memory
  preferences.begin("xb-set", true);
  TEAM_ID = preferences.getInt("teamid", 1234);
  preferences.end();

  Serial.begin(9600);
  delay(1000);

  Serial.println("\nSerial setup OK\n");

  // XBee Hardware serial
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  Serial2.println("\n$I SETUP OK!\n");

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

  Serial2.printf("Data transmission started on core %d\n", xPortGetCoreID());
  
  while(1)
  {
    // Wait for the transmission to be on
    while(TRANSMISSION_ON == 1)
    {
      // Fill transmission packet
      /*
      send_packet.TEAM_ID = "X";
      send_packet.MISSION_TIME = "X";
      send_packet.PACKET_COUNT = "X";
      send_packet.MODE = "X"; 
      send_packet.STATE = "X";
      send_packet.ALTITUDE = "X";
      send_packet.TEMPERATURE = "X";
      send_packet.PRESSURE = "X";
      send_packet.VOLTAGE = "X";
      send_packet.GYRO_R = "X";
      send_packet.GYRO_P = "X";
      send_packet.GYRO_Y = "X";
      send_packet.ACCEL_R = "X";
      send_packet.ACCEL_P = "X";
      send_packet.ACCEL_Y = "X";
      send_packet.MAG_R = "X";
      send_packet.MAG_P = "X";
      send_packet.MAG_Y = "X";
      send_packet.AUTO_GYRO_ROTATION_RATE = "X";
      send_packet.GPS_TIME = "X";
      send_packet.GPS_ALTITUDE = "X";
      send_packet.GPS_LATITUDE = "X";
      send_packet.GPS_LONGITUDE = "X";
      send_packet.GPS_SATS = "X";
      send_packet.CMD_ECHO = "X";
      */
    }
    // TODO: should mode be in flash memory?
    vTaskDelay(pdMS_TO_TICKS(25));
  }
} // END: void loop()

void receive_commands(void *pvParameters)
{
  
  Serial2.printf("Command reception started on core %d\n", xPortGetCoreID());

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
        safe_print("$W COMMAND REJECTED: LONGER THAN ALLOCATED BUFFER SIZE\n");
        continue;
      }

      // Extract data
      struct command_packet recv_packet;
      
      if(extract_cmd_msg(cmd_buff, &recv_packet) > 0)
      {
        safe_print("$W COMMAND REJECTED: FORMAT IS INCORRECT. EXPECTING 'CMD,<TEAM_ID>,<CMD>,<DATA>'\n");
        continue;
      }
      
      if(compare_strings(recv_packet.keyword, "CMD") == 0)
      {
        safe_print("$W REJECTED INPUT: NOT CMD\n");
        continue;
      }

      int team_id_chk_int;
      sscanf(recv_packet.team_id, "%d", &team_id_chk_int);
      if((team_id_chk_int != TEAM_ID) && (compare_strings(recv_packet.command, "RESET_TEAM_ID") == 0))
      {
        char msg_size = snprintf(NULL, 0, "$W REJECTED INPUT: TEAM ID DOES NOT MATCH. TEAM_ID is %d\n", TEAM_ID);
        char message[msg_size];
        snprintf(message, msg_size, "$W REJECTED INPUT: TEAM ID DOES NOT MATCH. TEAM_ID is %d\n", TEAM_ID);
        safe_print(message);
        continue;
      }

      // Parse command
      if(compare_strings(recv_packet.command, "CX")) do_cx(recv_packet.data);
      else if(compare_strings(recv_packet.command, "ST")) do_st(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIM")) do_sim(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIMP")) do_simp(recv_packet.data);
      else if(compare_strings(recv_packet.command, "CAL")) do_cal(recv_packet.data);
      else if(compare_strings(recv_packet.command, "MEC")) do_mec(recv_packet.data);
      else if(compare_strings(recv_packet.command, "RESET_TEAM_ID")) do_reset_team_id(recv_packet.data);
      else
      {
        safe_print("$W COMMAND REJECTED: NOT A COMMAND\n");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }
} // END: receive_commands()

void do_cx(const char *data)
{
  if(compare_strings(data, "ON"))
  {
    if(TRANSMISSION_ON == 0)
    {
      TRANSMISSION_ON = 1;
      timerWrite(send_timer, 0);
      timerAlarmEnable(send_timer);
      safe_print("$I RECEIVED COMMAND: BEGINNING PAYLOAD TRANSMISSION\n");
    }
    else
    {
      safe_print("$E TRANSMISSION IS ALREADY ON\n");
    }
  }
  else if(compare_strings(data, "OFF"))
  {
    if(TRANSMISSION_ON == 1)
    {
      timerAlarmDisable(send_timer);
      TRANSMISSION_ON = 0;
      safe_print("$I RECEIVED COMMAND: ENDING PAYLOAD TRANSMISSION\n");
    }
    else
    {
      safe_print("$E TRANSMISSION IS ALREADY OFF\n");
    }
  }
  else
  {
    safe_print("$E DATA IS NOT VALID; SEND ON/OFF");
  }
} // END: do_cx()

void do_st(const char *data)
{
  safe_print("$I RECEIVED COMMAND: SET TIME\n");
  if(compare_strings(data, "GPS"))
  {
    safe_print("$I GPS :=)\n");
    // TODO: Get time from GPS, pass this to MISSION_TIME
  }
  else if(time_format_check(data) == 0)
  {
    int i = 0;
    while(data[i] != '\0')
    {
      MISSION_TIME[i] = data[i];
      i++;
    }
    MISSION_TIME[i] = '\0';

    char message[SENTENCE_SIZE];

    snprintf(message, SENTENCE_SIZE, "$I SET TIME TO: %s\n", MISSION_TIME);
    safe_print(message);
  }
  else
  {
    safe_print("$E DATA IS NOT VALID. SEND EITHER UTC TIME OR 'GPS'\n");
  }
} // END: do_st()

void do_sim(const char *data)
{
  safe_print("$I RECEIVED COMMAND: SIMULATION MODE CONTROL\n");

  if(TRANSMISSION_ON == 1)
  {
    safe_print("$E SIMULATION MODE CANNOT BE CHANGED WHILE TRANSMISSION IS ON\n");
    return;
  }

  if(compare_strings(data, "ENABLE"))
  {
    switch(current_mode)
    {
      case SIM_OFF:
                  current_mode = SIM_READY;
                  safe_print("$I MODE SET: SIM_READY\n");
                  break;
      case SIM_READY:
                  safe_print("$I MODE IS ALREADY SET TO: SIM_READY\n");
                  break;
      case SIM_ON:
                  safe_print("$E SIM MODE IS ALREADY ACTIVATED!\n");
    }
  }
  else if(compare_strings(data, "ACTIVATE"))
  {
    switch(current_mode)
    {
      case SIM_READY:
                  safe_print("$I STARTING SIMULATION MODE...\n");
                  break;
      case SIM_OFF:
                  safe_print("$E SIMULATION MODE IS NOT ENABLED\n");
                  break;
      case SIM_ON:
                  safe_print("$E SIM MODE IS ALREADY ACTIVATED!\n");
    }
  }
  else if(compare_strings(data, "DISABLE"))
  {
    switch(current_mode)
    {
      case SIM_ON:
                  safe_print("$I ENDING SIMULATION MODE...\n");
                  current_mode = SIM_OFF;
                  break;
      case SIM_READY:
                  safe_print("$I ENDING SIMULATION MODE...\n");
                  current_mode = SIM_OFF;
                  break;
      case SIM_OFF:
                  safe_print("$I SIMULATION MODE ALREADY DISABLED.\n");
                  break;
    }
  }
  else
  {
    int msg_size = snprintf(NULL, 0, "$E UNRECOGNIZED SIM COMMAND: '%s'\n", data);
    char message[msg_size];
    snprintf(message, SENTENCE_SIZE, "$E UNRECOGNIZED SIM COMMAND: '%s'\n", data);
    safe_print(message);
  }
} // END: do_sim()

void do_simp(const char *data)
{
  // Check if we are in simulation mode
  if(current_mode != SIM_ON)
  {
      int msg_size = snprintf(NULL, 0, "$E NOT IN SIMULATION MODE; CODE: %d", current_mode) + 1;
      char message[msg_size];
      snprintf(message, msg_size, "$E NOT IS SIMULATION MODE; CODE: %d", current_mode);
      safe_print(message);
  }

  int sim_pressure_recv = atoi(data);

  // TODO: Do something with the data
} // END: do_simp()

void do_cal(const char *data)
{
  safe_print("$I RECEIVED COMMAND: CAL\n");

  // TODO: Calibrate altitude to 0 meters

  // TODO: Set up reset and processor recovery
} // END: do_Cal()

void do_mec(const char *data)
{
  safe_print("$I RECEIVED COMMAND: MEC\n");
  // TODO: complete after sensore are installed
} // END: do_mec()

void do_reset_team_id(const char *data)
{
  int new_team_id;
  sscanf(data, "%d", &new_team_id);
  
  // Write new team ID to flash memory
  preferences.begin("xb-set", false);
  preferences.putInt("teamid", new_team_id);
  TEAM_ID = preferences.getInt("teamid", 1234);
  preferences.end();

  int msg_size = snprintf(NULL, 0, "$I TEAM ID HAS BEEN RESET TO %d\n", TEAM_ID);
  char message[msg_size];
  snprintf(message, msg_size, "$I TEAM ID HAS BEEN RESET TO %d\n", TEAM_ID);
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
  char message[15] = "EXAMPLE DATA!\n";
  Serial2.println(message);
} // END: send_1()
