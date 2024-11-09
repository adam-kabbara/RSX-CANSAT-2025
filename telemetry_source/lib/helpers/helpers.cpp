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
           return 0;
       }  
       i++; 
    } 

    if(a[i] !='\0' || b[i] != '\0')
    {
       return 0;
    }
    
    return 1;
}

int time_format_check(const char *time_utc)
{
    // Check string is in format 'hh:mm:ss'
    for(int i = 0; i < 8; i++)
    {
        if(time_utc[i] == '\0') return 1;
    }
    if(time_utc[8] != '\0') return 1;

    for(int i = 0; i < 8; i++)
    {
        if(i != 2 && i != 5)
        {
            if(!isdigit(time_utc[i])) return 1;
        }
        else
        {
            if(time_utc[i] != ':') return 1;
        }
    }

    return 0;
}
