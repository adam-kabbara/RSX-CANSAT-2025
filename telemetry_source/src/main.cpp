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
char MISSION_TIME[GENERAL_WORD_SIZE] = "00:00:00";
mode_control current_mode = STANDBY;
TaskHandle_t recv_main;
TaskHandle_t send_main;
SemaphoreHandle_t print_mutex = xSemaphoreCreateMutex();
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
  preferences.begin("payload-settings", true);
  TEAM_ID = preferences.getInt("teamid", 1234);
  preferences.end();

  // Enable serial
  Serial.begin(9600);
  delay(1000);
  // XBee Hardware serial
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  Serial2.println("\n$I SETUP OK\n");

  xTaskCreatePinnedToCore(
    receive_commands,      // Function to be performed the task is called 
    "Receive Commands",    // Name of the task in text
    10000,          // Stack size (Memory size assigned to the task)
    NULL,         // Pointer that will be used as the parameter for the task being created
    1,           // Task Priority 
    &recv_main, // The task handler
    0          //xCoreID (Core 0)
  );

  xTaskCreatePinnedToCore(
    send_data,      // Function to be performed the task is called 
    "Send Data",    // Name of the task in text
    10000,          // Stack size (Memory size assigned to the task)
    NULL,         // Pointer that will be used as the parameter for the task being created
    1,           // Task Priority 
    &send_main, // The task handler
    1          //xCoreID (Core 1)
  );

  assert(print_mutex);
}

void loop()
{

}

// This function CANNOT write to ANY global parameters
void send_data(void *pvParameters)
{

  xSemaphoreTake(print_mutex, portMAX_DELAY);
  Serial2.printf("Command reception started on core %d\n", xPortGetCoreID());
  xSemaphoreGive(print_mutex);
  
  while(1)
  {
    // Wait for the mode to be FLIGHT or SIM
    // Then start sending data
    // timerAlarmEnable(send_timer);
    // TODO: should mode be in flash memory?
  }
}

void receive_commands(void *pvParameters)
{
  
  xSemaphoreTake(print_mutex, portMAX_DELAY);
  Serial2.printf("Command reception started on core %d\n", xPortGetCoreID());
  xSemaphoreGive(print_mutex);

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
        break;
      }

      // Extract data
      struct command_packet recv_packet;
      if(extract_cmd_msg(cmd_buff, &recv_packet) > 0)
      {
        safe_print("$W COMMAND REJECTED: FORMAT IS INCORRECT. EXPECTING 'CMD,<TEAM_ID>,<CMD>,<DATA>'\n");
        break;
      }
      if(compare_strings(recv_packet.keyword, "CMD") > 0)
      {
        safe_print("$W REJECTED INPUT: NOT CMD\n");
        break;
      }
      int team_id_chk_int;
      sscanf(recv_packet.team_id, "%d", &team_id_chk_int);
      if(team_id_chk_int != TEAM_ID && compare_strings(recv_packet.command, "RESET_TEAM_ID"))
      {
        safe_print("$W REJECTED INPUT: TEAM ID DOES NOT MATCH.\n");
        break;
      }

      // Parse command
      if(compare_strings(recv_packet.command, "CX") == 0) do_cx(recv_packet.data);
      else if(compare_strings(recv_packet.command, "ST") == 0) do_st(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIM") == 0) do_sim(recv_packet.data);
      else if(compare_strings(recv_packet.command, "SIMP") == 0) do_simp(recv_packet.data);
      else if(compare_strings(recv_packet.command, "CAL") == 0) do_cal(recv_packet.data);
      else if(compare_strings(recv_packet.command, "MEC") == 0) do_mec(recv_packet.data);
      else if(compare_strings(recv_packet.command, "RESET_TEAM_ID") == 0) do_reset_team_id(recv_packet.data);
      else
      {
        safe_print("$W COMMAND REJECTED: NOT A COMMAND\n");
      }

    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

}

void do_cx(const char *data)
{
  // TODO: Multi-thread for data sending and command reception
  // TODO: If CX OFF received set mode to STANDBY
  safe_print("$I RECEIVED COMMAND: BEGIN PAYLOAD TRANSMISSION\n");
}

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

    xSemaphoreTake(print_mutex, portMAX_DELAY);
    Serial2.printf("$I SET TIME TO: %s\n", MISSION_TIME);
    xSemaphoreGive(print_mutex);
  }
  else
  {
    safe_print("$E DATA IS NOT VALID. SEND EITHER UTC TIME OR 'GPS'\n");
  }
}

void do_sim(const char *data)
{
  safe_print("$I RECEIVED COMMAND: SIMULATION MODE CONTROL\n");
  if(compare_strings(data, "ENABLE"))
  {
    switch(current_mode)
    {
      case STANDBY:
                  current_mode = SIM_READY;
                  safe_print("$I MODE SET: SIM_READY\n");
                  break;
      case SIM_READY:
                  safe_print("$I MODE IS ALREADY SET TO: SIM_READY\n");
                  break;
      default:
                  safe_print("$E PAYLOAD IS NOT IN STANDBY MODE\n");
    }
  }
  else if(compare_strings(data, "ACTIVATE"))
  {
    switch(current_mode)
    {
      case SIM_READY:
                  safe_print("$I STARTING SIMULATION MODE...\n");
                  // TODO
                  break;
      case STANDBY:
                  safe_print("$E SIMULATION MODE IS NOT ENABLED\n");
                  break;
      default:
                  safe_print("$E PAYLOAD IS NOT IN STANDBY MODE\n");
    }
  }
  else if(compare_strings(data, "DISABLE"))
  {
    switch(current_mode)
    {
      case SIM:
                  safe_print("$I ENDING SIMULATION MODE...\n");
                  current_mode = STANDBY;
                  // TODO
                  break;
      case STANDBY:
                  current_mode = STANDBY;
                  safe_print("$I SIMULATION MODE DISABLED.\n");
                  break;
      case FLIGHT:
                  safe_print("$E PAYLOAD IS IN FLIGHT MODE\n");
                  break;
      default:
                  safe_print("$E PAYLOAD ON STANDBY\n");
    }
  }
  else
  {
    xSemaphoreTake(print_mutex, portMAX_DELAY);
    Serial2.println("$E UNRECOGNIZED SIM COMMAND: '" + String(data) + "'\n");
    xSemaphoreGive(print_mutex);
  }
}

void do_simp(const char *data)
{
  // Check if we are in simulation mode
  if(current_mode != SIM)
  {
    xSemaphoreTake(print_mutex, portMAX_DELAY);
    Serial2.printf("$E NOT IS SIMULATION MODE; CODE: %d", current_mode);
    xSemaphoreGive(print_mutex);
  }

  int sim_pressure_recv = atoi(data);

  // TODO: Do something with the data
}

void do_cal(const char *data)
{
  safe_print("$I RECEIVED COMMAND: CAL\n");

  // TODO: Calibrate altitude to 0 meters

  // TODO: Set up reset and processor recovery
}

void do_mec(const char *data)
{
  safe_print("$I RECEIVED COMMAND: MEC\n");
  // TODO: complete after sensore are installed
}

void do_reset_team_id(const char *data)
{
  int new_team_id;
  sscanf(data, "%d", &new_team_id);
  
  // Write new team ID to flash memory
  preferences.begin("payload-settings", false);
  preferences.putInt("teamid", new_team_id);
  TEAM_ID = preferences.getInt("teamid", 1234);
  preferences.end();

  xSemaphoreTake(print_mutex, portMAX_DELAY);
  Serial2.printf("$I TEAM ID HAS BEEN RESET TO %d\n", TEAM_ID);
  xSemaphoreGive(print_mutex);
}

void IRAM_ATTR send_1()
{
  // TODO: This will send a packet regardless if it is ready
}

void safe_print(const char *msg)
{
  xSemaphoreTake(print_mutex, portMAX_DELAY);
  Serial2.println(msg);
  xSemaphoreGive(print_mutex);
}