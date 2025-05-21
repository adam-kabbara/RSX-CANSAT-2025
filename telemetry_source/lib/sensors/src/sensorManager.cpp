#include "sensorManager.h"

OperatingState SensorManager::updateState(OperatingState curr_state, MissionManager &mission_info)
{
    int idx = alt_data.idx - 1;
    int size = alt_data.window_size;

    switch(curr_state)
    {
        case LAUNCH_PAD: {

            // Average of last 5 samples is > 5m

            float alt_sum = 0.0;

            for (int i = 0; i < 5; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 5;

            if(alt_sum > 5.0)
            {
                return ASCENT;
            }
            break;
        }
        
        case ASCENT: {
            // Because sim mode only sends reading every 1 second it's way harder to detect apogee
            if(mission_info.getOpMode() == OPMODE_SIM)
            {
                // Just check current reading is <10m difference from last reading
                if(alt_data.buffer[idx] - alt_data.buffer[(idx - 1 + size) % size] < 5)
                {
                    alt_data.max_alt = alt_data.buffer[idx];
                    return APOGEE;
                }
            }
            else
            {
                if(alt_data.sample_count > size)
                {
                    // To detect apogee, take last three averages of three samples at 0.15sec intervals
                    // and determine if the difference between each of them is less than 5m

                    int step = 0.15 * SENSOR_SAMPLE_RATE_HZ;

                    float avg0 = (alt_data.buffer[(idx + size - 1) % size] +
                                alt_data.buffer[idx] +
                                alt_data.buffer[(idx + 1) % size]) / 3.0;

                    float avg1 = (alt_data.buffer[(idx - step + size - 1) % size] +
                                alt_data.buffer[(idx - step + size) % size] +
                                alt_data.buffer[(idx - step + size + 1) % size]) / 3.0;

                    float avg2 = (alt_data.buffer[(idx - 2 * step + size - 1) % size] +
                                alt_data.buffer[(idx - 2 * step + size) % size] +
                                alt_data.buffer[(idx - 2 * step + size + 1) % size]) / 3.0;

                    float d1 = fabs(avg0 - avg1);
                    float d2 = fabs(avg1 - avg2);

                    if (d1 < 5.0 && d2 < 5.0)
                    {
                        alt_data.max_alt = avg1;
                        return APOGEE;
                    }
                }
            }
            break;
        }

        case APOGEE: {

            // Last three average of 3 sample reading over 0.25 sec intervals are decreasing

            int step = 0.25 * SENSOR_SAMPLE_RATE_HZ;

            float avg0 = (alt_data.buffer[(idx + size - 1) % size] +
                        alt_data.buffer[idx] +
                        alt_data.buffer[(idx + 1) % size]) / 3.0;

            float avg1 = (alt_data.buffer[(idx - step + size - 1) % size] +
                        alt_data.buffer[(idx - step + size) % size] +
                        alt_data.buffer[(idx - step + size + 1) % size]) / 3.0;

            float avg2 = (alt_data.buffer[(idx - 2 * step + size - 1) % size] +
                        alt_data.buffer[(idx - 2 * step + size) % size] +
                        alt_data.buffer[(idx - 2 * step + size + 1) % size]) / 3.0;

            // Replace max alt if needed
            if(avg0 > alt_data.max_alt)
            {
                alt_data.max_alt = avg0;
            }

            if (avg0 < avg1 && avg1 < avg2)
            {
                return DESCENT;
            }
            break;
        }

        case DESCENT: {
            // Average of last 5 readings are 75% of max

            float alt_sum = 0.0;

            for (int i = 0; i < 5; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 5;

            if (alt_sum <= 0.75 * alt_data.max_alt)
            {
                return PROBE_RELEASE;
            }
            break;
        }

        case PROBE_RELEASE: {

            // Average of last 5 samples is < 5m

            float alt_sum = 0.0;

            for (int i = 0; i < 5; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 5;

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
      send_packet.PRESSURE = mission_info.getSimpData()/1000.0;
    }
    else
    {
      send_packet.PRESSURE = getPressure();
    }

    // ALTITUDE
    send_packet.ALTITUDE = pressure_to_alt(send_packet.PRESSURE*10.0) - mission_info.getLaunchAlt();

    // STATE
    if(mission_info.getOpMode() == OPMODE_FLIGHT)
    {
        alt_data.buffer[alt_data.idx] = send_packet.ALTITUDE;
        alt_data.idx = (alt_data.idx + 1) % alt_data.window_size;
        alt_data.sample_count++;
    }

    mission_info.setOpState(updateState(mission_info.getOpState(), mission_info));

    strcpy(send_packet.STATE, op_state_to_string(mission_info.getOpState()));

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

void SensorManager::setAltData(float alt)
{
    alt_data.buffer[alt_data.idx] = alt;
    alt_data.idx = (alt_data.idx + 1) % alt_data.window_size;
    alt_data.sample_count++;
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
    alt_data.sample_count = 0;
}

void SensorManager::setPacketCount(int count)
{
    send_packet.PACKET_COUNT = count;
}

float SensorManager::getPressure()
{
    return 900.0;
}