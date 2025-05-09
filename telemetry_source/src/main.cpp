#include "includes.h"
#include "helpers.h"
#include "commandManager.h"
#include "serialManager.h"
#include "telemetry.h"
#include <Preferences.h>

hw_timer_t *send_timer = NULL;
SerialManager xbee_serial(Serial2);
struct mission_info_struct mission_info;
uint8_t send_flag = 0;
Preferences preferences;

void IRAM_ATTR send_1();

void setup()
{
    // Set timer to transmit at 1Hz
    send_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(send_timer, &send_1, true);
    timerAlarmWrite(send_timer, 1000000, true);

    xbee_serial.begin();

    // Clean flash memory
    // nvs_flash_erase(); // erase the NVS partition
    // nvs_flash_init(); // initialize the NVS partition

    // Remove all preferences under the opened namespace
    // mission_info.preferences.clear();

    // Reset sequence
    resetSeq(xbee_serial, mission_info, preferences);

    xbee_serial.sendInfoMsg("Startup Completed.");
}

void loop()
{
    char cmd_buff[CMD_BUFF_SIZE];
    CommandManager cmd_mgr;
    struct transmission_packet pckt;
    float alt_buffer[ALT_WINDOW_SIZE];
    int alt_buffer_indx = 0;
    float max_alt = 0.0;

    // Run this loop at 40Hz
    while(1)
    {
        // Just process commands in idle state
        // 10Hz
        while(mission_info.op_state == IDLE)
        {
            if(xbee_serial.get_data(cmd_buff))
            {
                if(cmd_mgr.processCommand(cmd_buff, xbee_serial, mission_info, preferences))
                {
                    cmd_buff_to_echo(cmd_buff, pckt.CMD_ECHO);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        xbee_serial.sendInfoMsg("MISSION STARTING!");

        // Exited IDLE state, turn on timer
        timerWrite(send_timer, 0);
        timerAlarmEnable(send_timer);

        alt_buffer_indx = 0;

        if(mission_info.op_mode == OPMODE_SIM)
        {
            xbee_serial.sendInfoMsg("BEGIN_SIMP");
        }

        // Process command and then check for sending data
        // 40 Hz
        while(mission_info.op_state != IDLE)
        {
            if(xbee_serial.get_data(cmd_buff))
            {
                if(cmd_mgr.processCommand(cmd_buff, xbee_serial, mission_info, preferences))
                {
                    cmd_buff_to_echo(cmd_buff, pckt.CMD_ECHO);
                }
            }

            // Wait until first simulation packet is received
            if(mission_info.op_mode == OPMODE_SIM && mission_info.FIRST_SIMP == 0)
            {
                continue;
            }

            sampleSensors(mission_info, pckt, alt_buffer, ALT_WINDOW_SIZE, alt_buffer_indx, max_alt);

            if(send_flag == 1)
            {
                mission_info.packet_count++;
                pckt.PACKET_COUNT = mission_info.packet_count;
                preferences.begin("xb-set", false);
                preferences.putInt("pckts", mission_info.packet_count);
                preferences.end();

                xbee_serial.sendTelemetry(&pckt);
                send_flag = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(25));
        }

        // Back to IDLE state, turn off timer, reset settings
        timerAlarmDisable(send_timer);
        mission_info.ALT_CAL_CHK = 0;
        mission_info.FIRST_SIMP = 0;
    }
}

void IRAM_ATTR send_1()
{
  send_flag = 1;
} // END: send_1()
