#include <Arduino.h>
#include "packet.h"
#include "global.h"
#include "helpers.h"

void IRAM_ATTR send_1();
void do_cx(char *data);
void do_st(char *data);
void do_sim(char *data);
void do_simp(char *data);
void do_cal(char *data);
void do_mec(char *data);
void do_reset_team_id(char *data);

hw_timer_t *send_timer = NULL;
Packet *pckt = new Packet();
char cmd_buff[CMD_BUFF_SIZE];
int TEAM_ID = 9999;

void setup() 
{

  // Set up 1Hz timer
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &send_1, true);
  timerAlarmWrite(send_timer, 1000000, true);

  // Enable serial
  Serial.begin(9600);
  delay(1000);
  // XBee Hardware serial
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(1000);

  Serial.println("SETUP OK");
}

void loop() 
{

  // Receive commands from ground station
  while(1)
  {
    if(Serial2.available())
    {
      byte bytes_read = Serial2.readBytesUntil('\n', cmd_buff, CMD_BUFF_SIZE - 1);
      cmd_buff[bytes_read] = '\0';

      if(bytes_read == CMD_BUFF_SIZE - 1)
      {
        Serial2.println("COMMAND REJECTED: LONGER THAN ALLOCATED BUFFER SIZE\n");
        break;
      }

      // Extract data
      struct command_packet recv_packet;
      if(extract_cmd_msg(cmd_buff, &recv_packet) > 0)
      {
        Serial2.println("COMMAND REJECTED: FORMAT IS INCORRECT. EXPECTING 'CMD,<TEAM_ID>,<CMD>,<DATA>'\n");
        break;
      }
      if(comapre_strings(recv_packet.keyword, "CMD") > 0)
      {
        Serial2.println("REJECTED INPUT: NOT CMD\n");
        break;
      }
      int team_id_chk_int;
      sscanf(recv_packet.team_id, "%d", &team_id_chk_int);
      if(team_id_chk_int != TEAM_ID && compare_strings(recv_packet.command, "RESET_TEAM_ID"))
      {
        Serial2.println("REJECTED INPUT: TEAM ID DOES NOT MATCH.\n");
        break;
      }

      // Parse command
      // TODO: Is using function pointer array faster? If else is messy but speed might be better
      if(compare_strings(msg, "CX") == 0) do_cx(recv_packet.data);
      else if(compare_strings(msg, "ST") == 0) do_st(recv_packet.data);
      else if(compare_strings(msg, "SIM") == 0) do_sim(recv_packet.data);
      else if(compare_strings(msg, "SIMP") == 0) do_simp(recv_packet.data);
      else if(compare_strings(msg, "CAL") == 0) do_cal(recv_packet.data);
      else if(compare_strings(msg, "MEC") == 0) do_mec(recv_packet.data);
      else if(compare_strings(msg, "RESET_TEAM_ID") == 0) do_reset_team_id(recv_packet.data);
      else
      {
        Serial2.println("COMMAND REJECTED: NOT A COMMAND\n");
      }

    }
  }

}

void do_cx(char *data)
{
  // TODO: Multi-thread for data sending and command reception
  Serial2.println("RECEIVED: CX\n");
}

void do_st(char *data)
{
  Serial2.println("RECEIVED: ST\n");
}

void do_simp(char *data)
{
  Serial2.println("RECEIVED: SIMP\n");
}

void do_cal(char *data)
{
  Serial2.println("RECEIVED: CAL\n");
}

void do_mec(char *data)
{
  Serial2.println("RECEIVED: MEC\n");
}

void do_reset_team_id(char *data)
{
  int new_team_id;
  sscanf(data, "%d", &new_team_id);
  TEAM_ID = new_team_id;
  Serial2.println("TEAM ID HAS BEEN RESET TO %d\n", new_team_id);
}

void IRAM_ATTR send_1()
{
  String send_msg;
  create_msg(pckt, send_msg);
  Serial.println("THIS WILL BE A PACKET");
}
