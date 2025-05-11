#include "includes.h"
#include "commandManager.h"
#include "serialManager.h"
#include "sensorManager.h"
#include "missionManager.h"

hw_timer_t *send_timer = NULL;
SerialManager xbee_serial(Serial2);
MissionManager mission_info;
uint8_t send_flag = 0;

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
    // xbee_serial.clearPref();

    // Reset sequence
    mission_info.resetSeq(xbee_serial);

    xbee_serial.sendInfoMsg("Startup Completed.");
}

void loop()
{
    char cmd_buff[CMD_BUFF_SIZE];
    char send_buffer[DATA_BUFF_SIZE];
    CommandManager cmd_mgr;
    SensorManager sensor_mgr;

    // Run this loop at 40Hz
    while(1)
    {
        // Just process commands in idle state
        // 10Hz
        while(mission_info.getOpState() == IDLE)
        {
            if(xbee_serial.get_data(cmd_buff))
            {
                if(cmd_mgr.processCommand(cmd_buff, xbee_serial, mission_info, sensor_mgr))
                {
                    sensor_mgr.cmd_buff_to_echo(cmd_buff);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        xbee_serial.sendInfoMsg("MISSION STARTING!");

        // Exited IDLE state, turn on timer
        timerWrite(send_timer, 0);
        timerAlarmEnable(send_timer);

        sensor_mgr.resetAltData();

        if(mission_info.getOpMode() == OPMODE_SIM)
        {
            xbee_serial.sendInfoMsg("BEGIN_SIMP");
        }

        // Process command and then check for sending data
        // 40 Hz
        while(mission_info.getOpState() != IDLE)
        {
            if(xbee_serial.get_data(cmd_buff))
            {
                if(cmd_mgr.processCommand(cmd_buff, xbee_serial, mission_info, sensor_mgr))
                {
                    sensor_mgr.cmd_buff_to_echo(cmd_buff);
                }
            }

            // Wait until first simulation packet is received
            if(mission_info.getOpMode() == OPMODE_SIM && mission_info.isWaitingSimp())
            {
                vTaskDelay(pdMS_TO_TICKS(25));
                continue;
            }

            sensor_mgr.sampleSensors(mission_info);

            if(send_flag == 1)
            {
                mission_info.incrPacketCount();
                sensor_mgr.setPacketCount(mission_info.getPacketCount());
                mission_info.beginPref("xb-set", false);
                mission_info.putPrefInt("pckts", mission_info.getPacketCount());
                mission_info.endPref();
                sensor_mgr.build_data_str(send_buffer, DATA_BUFF_SIZE);
                xbee_serial.sendTelemetry(send_buffer);
                send_flag = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(25));
        }

        // Back to IDLE state, turn off timer, reset settings
        timerAlarmDisable(send_timer);
        mission_info.setAltCalOff();
        mission_info.waitingForSimp();
    }
}

void IRAM_ATTR send_1()
{
  send_flag = 1;
} // END: send_1()
