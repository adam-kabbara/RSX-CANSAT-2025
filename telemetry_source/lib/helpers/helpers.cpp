#include "helpers.h"
#include "global.h"

int extract_cmd_msg(const char *buff, struct command_packet *packet)
{
    int delimiter_count = 0;
    int pos = 0;
    char *current_field = packet->keyword;

    for(int i = 0; buff[i] != '\0'; i++)
    {
        if(buff[i] == ',')
        {
            current_field[pos] = '\0';
            pos = 0;
            delimiter_count++;
            switch (delimiter_count)
            {
                case 1:
                    current_field = packet->team_id;
                    break;
                case 2:
                    current_field = packet->command;
                    break;
                case 3:
                    current_field = packet->data;
                    break;
                case 4:
                    return 1;
            }
        }
        else if(pos < WORD_SIZE - 1)
        {
            current_field[pos] = buff[i];
            pos++;
        }
        else
        {
            return 1;
        }
    }

    current_field[pos] = '\0';

    if(delimiter_count != 3)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int compare_strings(const char *a, const char *b)
{  
    int i = 0;

    while(a[i] !='\0' && b[i]!='\0')
    {  
       if(a[i]!=b[i])  
       {  
           return 1;
       }  
       i++; 
    } 

    if(a[i] !='\0' || b[i] != '\0')
    {
       return 1;
    }
    
    return 0;
}

int set_time_from_str(const char *time_utc)
{
    struct tm tm = {0};
    int hours, mins, secs;

    if(sscanf(time_utc, "%d:%d:%d", &hours, &mins, &secs) != 3)
    {
        return 1;
    }

    tm.tm_hour = hours;
    tm.tm_min = mins;
    tm.tm_sec = secs;

    setenv("TZ", "UTC0", 1);
    tzset();

    time_t t = mktime(&tm);

    struct timeval tv;
    tv.tv_sec = t;

    if(settimeofday(&tv, NULL) != 0)
    {
        return 1;
    }

    return 0;
}

const char* op_state_to_string(mission_info_struct::OperatingState state) 
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

const char* op_mode_to_string(mission_info_struct::OperatingMode mode, int full) 
{
    if (full == 1)
    {
        if(mode == mission_info_struct::OperatingMode::FLIGHT)
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
        if(mode == mission_info_struct::OperatingMode::FLIGHT)
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
    int i = 0;
    
    while (cmd_buff[i] != '\0') 
    {
        if (cmd_buff[i] == ',') 
        {
            cmd_echo[i] = '-';
        } else 
        {
            cmd_echo[i] = cmd_buff[i];
        }
        i++;
    }
    
    cmd_echo[i] = '\0';
}

char* build_data_str(struct transmission_packet *packet)
{
    static char buff[DATA_BUFF_SIZE];

    snprintf(buff, DATA_BUFF_SIZE,
        "%d,%s,%d,%s,%s," //5
        "%.1f,%.1f,%.1f,%.1f," //4
        "%d,%d,%d,%d,%d," //5
        "%d,%d,%d,%d,%d," //5
        "%s,%.1f,%.4f," //3
        "%.4f,%d,%s", //3
        packet->TEAM_ID, 
        packet->MISSION_TIME, 
        packet->PACKET_COUNT, 
        packet->MODE, 
        packet->STATE,
        packet->ALTITUDE, 
        packet->TEMPERATURE, 
        (packet->PRESSURE) / 1000, 
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
        packet->GPS_TIME, //20 
        packet->GPS_ALTITUDE, 
        packet->GPS_LATITUDE, 
        packet->GPS_LONGITUDE, 
        packet->GPS_SATS, 
        packet->CMD_ECHO);
    
    return buff;
}
