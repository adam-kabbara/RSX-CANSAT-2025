#ifndef PACKET_H
#define PACKET_H

typedef struct cansat_packet {
    String TEAM_ID;
    String MISSION_TIME;
    int PACKET_COUNT;
    String MODE;
    String STATE;
    float ALTITUDE;
    float TEMPERATURE;
    float PRESSURE;
    float VOLTAGE;
    float GYRO_R, GYRO_P, GYRO_Y;
    float ACCEL_R, ACCEL_P, ACCEL_Y;
    float MAG_R, MAG_P, MAG_Y;
    float AUTO_GYRO_ROTATION_RATE;
    float GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS;
    String CMD_ECHO;
} Packet;

void create_msg(const Packet *pckt, String &pckt_msg)
{
    pckt_msg = pckt->TEAM_ID + "," +
               pckt->MISSION_TIME + "," +
               String(pckt->PACKET_COUNT) + "," +
               pckt->MODE + "," +
               pckt->STATE + "," +
               String(pckt->ALTITUDE) + "," +
               String(pckt->TEMPERATURE) + "," +
               String(pckt->PRESSURE) + "," +
               String(pckt->VOLTAGE) + "," +
               String(pckt->GYRO_R) + "," +
               String(pckt->GYRO_P) + "," +
               String(pckt->GYRO_Y) + "," +
               String(pckt->ACCEL_R) + "," +
               String(pckt->ACCEL_P) + "," +
               String(pckt->ACCEL_Y) + "," +
               String(pckt->MAG_R) + "," +
               String(pckt->MAG_P) + "," +
               String(pckt->MAG_Y) + "," +
               String(pckt->AUTO_GYRO_ROTATION_RATE) + "," +
               String(pckt->GPS_TIME) + "," +
               String(pckt->GPS_ALTITUDE) + "," +
               String(pckt->GPS_LATITUDE) + "," +
               String(pckt->GPS_LONGITUDE) + "," +
               String(pckt->GPS_SATS) + "," +
               pckt->CMD_ECHO;
}

#endif /* PACKET_H */