#include "global.h"
#include "helpers.h"

/*------- DEFINITIONS ------*/
#define RX_PIN 16
#define TX_PIN 17
/*------- DEFINITIONS ------*/

/*------- FUNCTIONS ------*/
void IRAM_ATTR send_1();
void do_cx(const char *data);
void do_st(const char *data);
void do_sim(const char *data);
void do_simp(const char *data);
void do_cal(const char *data);
void do_mec(const char *data);
void do_reset_team_id(const char *data);
/*------- FUNCTIONS ------*/

/*------- VARIABLES ------*/
hw_timer_t *send_timer = NULL;
char cmd_buff[CMD_BUFF_SIZE];
int TEAM_ID = 9999;
char MISSION_TIME[GENERAL_WORD_SIZE] = "00:00:00";
mode_control current_mode = STANDBY;
/*------- VARIABLES ------*/

void setup() 
{

  // Set timer to transmit at 1Hz
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &send_1, true);
  timerAlarmWrite(send_timer, 1000000, true);

  // Enable serial
  Serial.begin(9600);
  delay(1000);
  // XBee Hardware serial
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  Serial2.println("\n$I SETUP OK\n");
}

void loop() 
{

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
        Serial2.println("$W COMMAND REJECTED: LONGER THAN ALLOCATED BUFFER SIZE\n");
        break;
      }

      // Extract data
      struct command_packet recv_packet;
      if(extract_cmd_msg(cmd_buff, &recv_packet) > 0)
      {
        Serial2.println("$W COMMAND REJECTED: FORMAT IS INCORRECT. EXPECTING 'CMD,<TEAM_ID>,<CMD>,<DATA>'\n");
        break;
      }
      if(compare_strings(recv_packet.keyword, "CMD") > 0)
      {
        Serial2.println("$W REJECTED INPUT: NOT CMD\n");
        break;
      }
      int team_id_chk_int;
      sscanf(recv_packet.team_id, "%d", &team_id_chk_int);
      if(team_id_chk_int != TEAM_ID && compare_strings(recv_packet.command, "RESET_TEAM_ID"))
      {
        Serial2.println("$W REJECTED INPUT: TEAM ID DOES NOT MATCH.\n");
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
        Serial2.println("$W COMMAND REJECTED: NOT A COMMAND\n");
      }

    }
  }

}

void do_cx(const char *data)
{
  // TODO: Multi-thread for data sending and command reception
  // TODO: If CX OFF received set mode to STANDBY
  Serial2.println("$I RECEIVED COMMAND: BEGIN PAYLOAD TRANSMISSION\n");
}

void do_st(const char *data)
{
  Serial2.println("$I RECEIVED COMMAND: SET TIME\n");
  if(compare_strings(data, "GPS"))
  {
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

    Serial2.printf("$I SET TIME TO: %s\n", MISSION_TIME);
  }
  else
  {
    Serial2.println("$E DATA IS NOT VALID. SEND EITHER UTC TIME OR 'GPS'\n");
  }
}

void do_sim(const char *data)
{
  Serial2.println("$I RECEIVED COMMAND: SIMULATION MODE CONTROL\n");
  if(compare_strings(data, "ENABLE"))
  {
    switch(current_mode)
    {
      case STANDBY:
                  current_mode = SIM_READY;
                  Serial2.println("$I MODE SET: SIM_READY\n");
                  break;
      case SIM_READY:
                  Serial2.println("$I MODE IS ALREADY SET TO: SIM_READY\n");
                  break;
      default:
                  Serial2.println("$E PAYLOAD IS NOT IN STANDBY MODE\n");
    }
  }
  else if(compare_strings(data, "ACTIVATE"))
  {
    switch(current_mode)
    {
      case SIM_READY:
                  Serial2.println("$I STARTING SIMULATION MODE...\n");
                  // TODO
                  break;
      case STANDBY:
                  Serial2.println("$E SIMULATION MODE IS NOT ENABLED\n");
                  break;
      default:
                  Serial2.println("$E PAYLOAD IS NOT IN STANDBY MODE\n");
    }
  }
  else if(compare_strings(data, "DISABLE"))
  {
    switch(current_mode)
    {
      case SIM:
                  Serial2.println("$I ENDING SIMULATION MODE...\n");
                  // TODO
                  break;
      case FLIGHT:
                  Serial2.println("$E PAYLOAD IS IN FLIGHT MODE\n");
                  break;
      default:
                  Serial2.println("$E PAYLOAD IS READY/ON STANDBY\n");
    }
  }
  else
  {
    Serial2.println("$E UNRECOGNIZED SIM COMMAND: '" + String(data) + "'\n");
  }
}

void do_simp(const char *data)
{
  Serial2.println("$I RECEIVED COMMAND: SIMP\n");
}

void do_cal(const char *data)
{
  Serial2.println("$I RECEIVED COMMAND: CAL\n");
  // TODO: Set up watchdog timer/critical state flash memory to recover from hanging/unexpected reset
}

void do_mec(const char *data)
{
  Serial2.println("$I RECEIVED COMMAND: MEC\n");
  // TODO: complete after sensore are installed
}

void do_reset_team_id(const char *data)
{
  int new_team_id;
  sscanf(data, "%d", &new_team_id);
  TEAM_ID = new_team_id;
  Serial2.printf("TEAM ID HAS BEEN RESET TO %d\n", new_team_id);
}

void IRAM_ATTR send_1()
{
  // TODO: This will send a packet regardless if it is ready
}
