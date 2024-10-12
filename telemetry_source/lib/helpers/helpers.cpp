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
                    return 0;
            }
        }
        else if(pos < CMD_WORD_SIZE - 1)
        {
            current_field[pos] = buff[i];
            pos++;
        }
        else
        {
            return 1;
        }
    }

    return 1;
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