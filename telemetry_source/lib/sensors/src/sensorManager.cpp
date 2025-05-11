#include "sensorManager.h"

OperatingState SensorManager::updateState(OperatingState curr_state)
{
    int idx = alt_data.idx;
    int size = alt_data.window_size;

    float alt_sum = 0.0;

    for(int i = 0; i < size; i++)
    {
        alt_sum += alt_data.buffer[i];
    }
    alt_sum = alt_sum / alt_data.window_size;

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
            float d1 = fabs(alt_data.buffer[idx] - alt_data.buffer[(idx - 1 + size) % size]);
            float d2 = fabs(alt_data.buffer[(idx - 1 + size) % size] - alt_data.buffer[(idx - 2 + size) % size]);
            float d3 = fabs(alt_data.buffer[(idx - 2 + size) % size] - alt_data.buffer[(idx - 3 + size) % size]);
            
            if (d1 < 5.0 && d2 < 5.0 && d3 < 5.0) // Change threshold as needed
            {
                alt_data.max_alt = alt_data.buffer[idx];
                return APOGEE;
            }
            break;
        }

        case APOGEE: {
            // last few readings are decreasing
            float a = alt_data.buffer[idx];
            float b = alt_data.buffer[(idx - 1 + size) % size];
            float c = alt_data.buffer[(idx - 2 + size) % size];

            if (a < b && b < c)
            {
                return DESCENT;
            }
            break;
        }

        case DESCENT: {
            // last few readings are 75% of max
            if (alt_sum <= 0.75 * alt_data.max_alt)
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

void SensorManager::sampleSensors(MissionManager &mission_info)
{
    // PRESSURE
    if(mission_info.getOpMode() == OPMODE_SIM)
    {
      send_packet.PRESSURE = mission_info.getSimpData();
    }
    else
    {
      send_packet.PRESSURE = random(0, 10000);
    }

    // ALTITUDE
    send_packet.ALTITUDE = pressure_to_alt(send_packet.PRESSURE/100) - mission_info.getLaunchAlt();

    // STATE
    alt_data.buffer[alt_data.idx] = send_packet.ALTITUDE;
    alt_data.idx = (alt_data.idx + 1) % alt_data.window_size;

    mission_info.setOpState(updateState(mission_info.getOpState()));

    // TEAM ID
    send_packet.TEAM_ID_PCKT = TEAM_ID;

    // MISSION TIME
    strcpy(send_packet.MISSION_TIME, "00:00:00");

    // MODE
    strcpy(send_packet.MODE, op_mode_to_string(mission_info.getOpMode(), 0));

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

void SensorManager::build_data_str(char *buff, size_t size)
{
    snprintf(buff, size,
        "%d,%s,%d,%s,%s," //5
        "%.1f,%.1f,%.1f,%.1f," //4
        "%d,%d,%d,%d,%d," //5
        "%d,%d,%d,%d,%d," //5
        "%s,%.1f,%.4f," //3
        "%.4f,%d,%s", //3
        send_packet.TEAM_ID_PCKT, 
        send_packet.MISSION_TIME, 
        send_packet.PACKET_COUNT, 
        send_packet.MODE, 
        send_packet.STATE,
        send_packet.ALTITUDE, 
        send_packet.TEMPERATURE, 
        send_packet.PRESSURE, 
        send_packet.VOLTAGE, 
        send_packet.GYRO_R, 
        send_packet.GYRO_P, 
        send_packet.GYRO_Y, 
        send_packet.ACCEL_R, 
        send_packet.ACCEL_P, 
        send_packet.ACCEL_Y, 
        send_packet.MAG_R, 
        send_packet.MAG_P, 
        send_packet.MAG_Y, 
        send_packet.AUTO_GYRO_ROTATION_RATE, 
        send_packet.GPS_TIME,
        send_packet.GPS_ALTITUDE, 
        send_packet.GPS_LATITUDE, 
        send_packet.GPS_LONGITUDE, 
        send_packet.GPS_SATS, 
        send_packet.CMD_ECHO);
    
}

const char* SensorManager::op_state_to_string(OperatingState state) 
{
    static const char* states[] = {
        "LAUNCH_PAD",
        "ASCENT",
        "APOGEE",
        "DESCENT",
        "PROBE_RELEASE",
        "LANDED",
        "IDLE"
    };

    return states[state];

}

const char* SensorManager::op_mode_to_string(OperatingMode mode, int full) 
{
    if (full == 1)
    {
        if(mode == OPMODE_FLIGHT)
        {
            return "FLIGHT";
        }
        else
        {
            return "SIM";
        }
    }
    else
    {
        if(mode == OPMODE_FLIGHT)
        {
            return "F";
        }
        else
        {
            return "S";
        }
    }
}

float SensorManager::pressure_to_alt(const float pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}


void SensorManager::cmd_buff_to_echo(char *cmd_buff) 
{
    int comma = 0;
    int echo_indx = 0;

    for (int i = 0; cmd_buff[i] != '\0'; i++)
    {
        if (cmd_buff[i] == ',')
        {
            comma++;
            continue;
        }

        if (comma > 1)
        {
            send_packet.CMD_ECHO[echo_indx++] = cmd_buff[i];
        }
    }

    send_packet.CMD_ECHO[echo_indx] = '\0';
}

void SensorManager::resetAltData()
{
    for(int i = 0; i < alt_data.window_size; i++)
    {
        alt_data.buffer[i] = 0.0;
    }
    alt_data.idx = 0;
    alt_data.current_alt = 0.0;
    alt_data.max_alt = 0.0;
}

void SensorManager::setPacketCount(int count)
{
    send_packet.PACKET_COUNT = count;
}

float SensorManager::getPressure()
{
    return 900.0;
}