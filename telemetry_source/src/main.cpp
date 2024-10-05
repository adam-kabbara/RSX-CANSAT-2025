#include <Arduino.h>
#include "packet.h"
hw_timer_t *send_timer = NULL;
Packet *pckt = new Packet();
void IRAM_ATTR send_1();

//TODO: Change all strings to char arrays

void setup() 
{

  // Set up 1Hz timer
  send_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(send_timer, &send_1, true);
  timerAlarmWrite(send_timer, 1000000, true);

  // Enable serial communication
  Serial.begin(9600);
  delay(1000);
  Serial.println("SETUP OK");
}

void loop() 
{

  // Wait for command to start sending data
  while(1)
  {
    if(Serial.available())
    {
      String recv = Serial.readString();
      if(!recv.equals("CXON"))
      {
        Serial.println("\nWARNING: Received unexpected command: " + recv + "\n");
        continue;
      }
      else
      {
        Serial.println("\nCOMMAND RECEIVED! Beginning communication....\n");
        // Add collection of data here
        // Send first packet manually, then start timer
        timerAlarmEnable(send_timer);
        while(1)
        {

        }
      }
    }
  }

}

void IRAM_ATTR send_1()
{
  String send_msg;
  create_msg(pckt, send_msg);
  Serial.println("THIS WILL BE A PACKET");
}

