#include "includes.h"
#include "commandManager.h"
#include "serialManager.h"
#include "sensorManager.h"
#include "missionManager.h"

hw_timer_t *send_timer = NULL;
SerialManager xbee_serial(Serial2);
SensorManager sensor_mgr;
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

    // Debug serial
    //Serial.begin(9600);

    // Setup SPIFF
    if(!LittleFS.begin(true))
    {
        xbee_serial.sendErrorMsg("An Error has occurred while mounting LittleFS!");
        return;
    }

    // Clean flash memory
    // nvs_flash_erase(); // erase the NVS partition
    // nvs_flash_init(); // initialize the NVS partition

    // Remove all preferences under the opened namespace
    // xbee_serial.clearPref();

    // Reset sequence
    mission_info.resetSeq(xbee_serial);

    sensor_mgr.startSensors(xbee_serial, mission_info);

    xbee_serial.sendInfoMsg("Startup Completed.");

    //Serial.println("\nStartup Completed.");
}

void loop()
{
    char cmd_buff[CMD_BUFF_SIZE];
    char send_buffer[DATA_BUFF_SIZE];
    CommandManager cmd_mgr;
    int delay_rate_ms = (1/SENSOR_SAMPLE_RATE_HZ) * 1000;
    int cannot_write_to_file = 0;

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

        sensor_mgr.resetAltData();

        if(mission_info.getOpMode() == OPMODE_SIM)
        {
            xbee_serial.sendInfoMsg("BEGIN_SIMP");
        }

        if(mission_info.getOpState() == LAUNCH_PAD && LittleFS.exists("/logs.txt")) 
        {
            xbee_serial.sendInfoMsg("Removing logs.txt from file system.");
            LittleFS.remove("/logs.txt");
        }

        File logfile = LittleFS.open("/logs.txt", FILE_APPEND);
        if(!logfile)
        {
            xbee_serial.sendErrorMsg("There was an error opening a file for saving data!");
        }

        // Start timer and camera
        digitalWrite(CAMERA1_SIGNAL_PIN, HIGH);
        digitalWrite(CAMERA2_SIGNAL_PIN, HIGH);
        timerWrite(send_timer, 0);
        timerAlarmEnable(send_timer);

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
                vTaskDelay(pdMS_TO_TICKS(delay_rate_ms));
                continue;
            }

            sensor_mgr.sampleSensors(mission_info, xbee_serial);

            if(send_flag == 1)
            {
                mission_info.incrPacketCount();
                sensor_mgr.setPacketCount(mission_info.getPacketCount());
                mission_info.beginPref("xb-set", false);
                mission_info.putPrefInt("pckts", mission_info.getPacketCount());
                mission_info.endPref();
                sensor_mgr.build_data_str(send_buffer, DATA_BUFF_SIZE);
                xbee_serial.sendTelemetry(send_buffer);
                if(logfile.size() < MAX_LOG_FILE_SIZE_BYTES)
                {
                    logfile.println(send_buffer);
                    //logfile.flush(); // idk
                }
                else
                {
                    if(cannot_write_to_file == 0)
                    {
                        xbee_serial.sendErrorMsg("Could not write to logfile, size exceeds 1MB. Data will not be written until mission is restarted.");
                    }
                    cannot_write_to_file = 1;
                }
                send_flag = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(delay_rate_ms));
        }

        // Back to IDLE state, turn off timer & camera, reset settings
        timerAlarmDisable(send_timer);
        digitalWrite(CAMERA1_SIGNAL_PIN, LOW);
        digitalWrite(CAMERA2_SIGNAL_PIN, LOW);
        mission_info.setAltCalOff();
        mission_info.waitingForSimp();
        logfile.close();
        cannot_write_to_file = 0;
    }
}

void IRAM_ATTR send_1()
{
  send_flag = 1;
} // END: send_1()
