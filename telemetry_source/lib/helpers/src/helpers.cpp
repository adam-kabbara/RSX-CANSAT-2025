#include "helpers.h"

const char* op_state_to_string(OperatingState state) 
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

const char* op_mode_to_string(OperatingMode mode, int full) 
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


void cmd_buff_to_echo(char *cmd_buff, char *cmd_echo) 
{
    int comma = 0;
    int value = 0;
    int echo_indx = 0;
    int i = 0;
    
    for(; cmd_buff[i] != '\0'; i++)
    {
        if(cmd_buff[i] == ',')
        {
            comma++;
            continue;
        }
        if(comma == 2 || comma == 3)
        {
            cmd_echo[echo_indx] = cmd_buff[i];
            echo_indx++;
        }
    }
    
    cmd_echo[i] = '\0';
}

void build_data_str(const struct transmission_packet *packet, char *buff, size_t size)
{

    snprintf(buff, size,
        "%d,%s,%d,%s,%s," //5
        "%.1f,%.1f,%.1f,%.1f," //4
        "%d,%d,%d,%d,%d," //5
        "%d,%d,%d,%d,%d," //5
        "%s,%.1f,%.4f," //3
        "%.4f,%d,%s", //3
        packet->TEAM_ID_PCKT, 
        packet->MISSION_TIME, 
        packet->PACKET_COUNT, 
        packet->MODE, 
        packet->STATE,
        packet->ALTITUDE, 
        packet->TEMPERATURE, 
        packet->PRESSURE, 
        packet->VOLTAGE, 
        packet->GYRO_R, 
        packet->GYRO_P, 
        packet->GYRO_Y, 
        packet->ACCEL_R, 
        packet->ACCEL_P, 
        packet->ACCEL_Y, 
        packet->MAG_R, 
        packet->MAG_P, 
        packet->MAG_Y, 
        packet->AUTO_GYRO_ROTATION_RATE, 
        packet->GPS_TIME,
        packet->GPS_ALTITUDE, 
        packet->GPS_LATITUDE, 
        packet->GPS_LONGITUDE, 
        packet->GPS_SATS, 
        packet->CMD_ECHO);
    
}

float pressure_to_alt(const float pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}
