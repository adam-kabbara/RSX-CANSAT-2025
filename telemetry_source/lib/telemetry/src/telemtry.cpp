#include "telemetry.h"

void resetSeq(SerialManager &serial, mission_info_struct &mission_info, Preferences preferences)
{
    esp_reset_reason_t reset_reason = esp_reset_reason();

    serial.sendErrorMsg("Processor Restarted!");

    switch (reset_reason) {

        case ESP_RST_SW:
            serial.sendErrorMsg("Reset Reason: Manual software trigger");
            break;
        case ESP_RST_UNKNOWN:
            serial.sendErrorMsg("Reset Reason: Unknown");
            break;
        case ESP_RST_POWERON:
            serial.sendErrorMsg("Reset Reason: Power On Reset");
            break;
        case ESP_RST_EXT:
            serial.sendErrorMsg("Reset Reason: External Reset");
            break;
        case ESP_RST_PANIC:
            serial.sendErrorMsg("Reset Reason: Software Reset due to Panic/Exception");
            break;
        case ESP_RST_INT_WDT:
            serial.sendErrorMsg("Reset Reason: Interrupt Watchdog Reset");
            break;
        case ESP_RST_TASK_WDT:
            serial.sendErrorMsg("Reset Reason: Task Watchdog Reset");
            break;
        case ESP_RST_WDT:
            serial.sendErrorMsg("Reset Reason: General Watchdog Reset");
            break;
        case ESP_RST_DEEPSLEEP:
            serial.sendErrorMsg("Reset Reason: Deep Sleep Wakeup");
            break;
        case ESP_RST_BROWNOUT:
            serial.sendErrorMsg("Reset Reason: Brownout Reset");
            break;
        case ESP_RST_SDIO:
            serial.sendErrorMsg("Reset Reason: SDIO Reset");
            break;
        default:
            serial.sendErrorMsg("Reset Reason: Unknown");
            break;
    }

    preferences.begin("xb-set", true);
    mission_info.op_state = static_cast<OperatingState>(preferences.getInt("opstate", 6));

    if(mission_info.op_state != IDLE)
    {
        serial.sendErrorMsg("Perfoming recovery as processor was not in IDLE state! Telemetry should resume!");
        // Get packet count, launch altitude
        mission_info.launch_altitude = preferences.getFloat("grndalt", 0.0);
        mission_info.sim_status = static_cast<SimModeStatus>(preferences.getInt("simst", 0));
        mission_info.op_mode = static_cast<OperatingMode>(preferences.getInt("opmode", 0));
        mission_info.packet_count = preferences.getInt("pckts", 0);
    }

    preferences.end();
}

void sampleSensors(mission_info_struct &mission_info, transmission_packet &send_packet, float *alt_buff, size_t size, int &idx, float &max_alt)
{
    // PRESSURE
    if(mission_info.op_mode == OPMODE_SIM)
    {
      send_packet.PRESSURE = mission_info.SIMP_DATA;
    }
    else
    {
      send_packet.PRESSURE = random(0, 10000);
    }

    // ALTITUDE
    send_packet.ALTITUDE = pressure_to_alt(send_packet.PRESSURE/100) - mission_info.launch_altitude;

    // STATE

    // Calculate moving average over last three samples to reduce error
    alt_buff[idx] = send_packet.ALTITUDE;
    idx = (idx + 1) % ALT_WINDOW_SIZE;

    mission_info.op_state = updateState(mission_info.op_state, alt_buff, size, idx, max_alt);

    // TEAM ID
    send_packet.TEAM_ID_PCKT = TEAM_ID;

    // MISSION TIME
    strcpy(send_packet.MISSION_TIME, "00:00:00");

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
}

OperatingState updateState(OperatingState curr_state, float *alt_buff, size_t size, int idx, float &max_alt)
{
    float alt_sum = 0;

    for(int i = 0; i < size; i++)
    {
        alt_sum += alt_buff[i];
    }
    alt_sum = alt_sum / size;

    switch(curr_state)
    {
        case LAUNCH_PAD: {
            if(alt_sum > 5.0)
            {
                return ASCENT;
            }
            break;
        }
        
        case ASCENT: {
            // Check if the last 3 altitude changes are small (altitude stabilizing)
            float d1 = fabs(alt_buff[idx] - alt_buff[(idx - 1 + size) % size]);
            float d2 = fabs(alt_buff[(idx - 1 + size) % size] - alt_buff[(idx - 2 + size) % size]);
            float d3 = fabs(alt_buff[(idx - 2 + size) % size] - alt_buff[(idx - 3 + size) % size]);
            
            if (d1 < 5.0 && d2 < 5.0 && d3 < 5.0) // Change threshold as needed
            {
                max_alt = alt_buff[idx];
                return APOGEE;
            }
            break;
        }

        case APOGEE: {
            // last few readings are decreasing
            float a = alt_buff[idx];
            float b = alt_buff[(idx - 1 + size) % size];
            float c = alt_buff[(idx - 2 + size) % size];

            if (a < b && b < c)
            {
                return DESCENT;
            }
            break;
        }

        case DESCENT: {
            // last few readings are 75% of max
            if (alt_sum <= 0.75 * max_alt)
            {
                return PROBE_RELEASE;
            }
            break;
        }

        case PROBE_RELEASE: {
            if(alt_sum < 5.0)
            {
                return LAUNCH_PAD;
            }
            break;
        }

    }

    return curr_state;
}
