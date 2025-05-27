#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "includes.h"
#include "serialManager.h"
#include "missionManager.h"
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

class SensorManager
{
private:

    typedef struct transmission_packet {
        int TEAM_ID_PCKT = 0;
        char MISSION_TIME[DATA_SIZE] = "";
        int PACKET_COUNT = 0;
        char MODE[2] = ""; 
        char STATE[DATA_SIZE] = "";
        float ALTITUDE = 0.0;
        float TEMPERATURE = 0.0;
        float PRESSURE = 0.0;
        float VOLTAGE = 0.0;
        int GYRO_R = 0;
        int GYRO_P = 0;
        int GYRO_Y = 0;
        int ACCEL_R = 0;
        int ACCEL_P = 0;
        int ACCEL_Y = 0;
        int MAG_R = 0;
        int MAG_P = 0;
        int MAG_Y = 0;
        int AUTO_GYRO_ROTATION_RATE = 0;
        char GPS_TIME[DATA_SIZE] = "";
        float GPS_ALTITUDE = 0.0;
        float GPS_LATITUDE = 0.0;
        float GPS_LONGITUDE = 0.0;
        int GPS_SATS = 0;
        char CMD_ECHO[CMD_BUFF_SIZE] = "";
        int CAMERA_STATUS = 0;
    } transmission_packet;

    transmission_packet send_packet;

    typedef struct altitude_data {
        size_t window_size = ALTITUDE_WINDOW_SIZE;
        float buffer[ALTITUDE_WINDOW_SIZE] = {0};
        int idx = 0;
        float current_alt = 0.0;
        float max_alt = 0.0;
        int sample_count = 0;
    } altitude_data;

    altitude_data alt_data;

    Adafruit_BME280 bme;
    Servo m_servo_release;
    Servo m_servo_gyro_1;
    Servo m_servo_gyro_2;
    Servo m_servo_camera;

    int servo_1_pos = 0;

public:

    OperatingState updateState(OperatingState curr_state, MissionManager &mission_info);

    void sampleSensors(MissionManager &mission_info);

    void build_data_str(char *buff, size_t size);

    float pressure_to_alt(const float pressure);

    const char* op_mode_to_string(OperatingMode mode, int full);

    const char* op_state_to_string(OperatingState state);

    void cmd_buff_to_echo(char *cmd_buff);

    void setPacketCount(int count);

    void resetAltData();

    float getPressure();

    float getTemp();

    void setAltData(float alt);

    void startSensors(SerialManager &ser);

    void writeReleaseServo(int pos);

    void writeGyroServo1(int pos);

    void writeGyroServo2(int pos);

    void writeCameraServo(int pos);

    float getGpsAlt();

    float getGpsLat();

    float getGpsLong();

    int getGpsSats();

    void getRtcTime(char time_str[DATA_SIZE]);

    void getGpsTime(char time_str[DATA_SIZE]);
};

#endif /* SENSOR_MANAGER_H */