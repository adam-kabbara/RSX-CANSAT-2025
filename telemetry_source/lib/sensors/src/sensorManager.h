#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "includes.h"
#include "serialManager.h"
#include "missionManager.h"

#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <uRTCLib.h>
#include "pid.h"

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
        float MAG_R = 0.0;
        float MAG_P = 0.0;
        float MAG_Y = 0.0;
        float AUTO_GYRO_ROTATION_RATE = 0.0;
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
    
    HardwareSerial *GPS_Serial;

    Adafruit_BME280 bme;
    TinyGPSPlus gps;

    Servo m_servo_release;
    Servo m_servo_gyro_1;
    Servo m_servo_gyro_2;
    Servo m_servo_camera;

    SPIClass *mySPI;
    Adafruit_BNO08x *bno08x;
    Adafruit_LIS3MDL lis3mdl;

    uRTCLib *rtc;

    float currRPM = 0.0;
    float prevRPM = 0.0;
    int lastState = HIGH;
    unsigned long lastPulseTime = 0;
    unsigned long pulseInterval = 0;

    PIDController pid_cntl;
    float ax = 0;
    float ay = 0;
    float az = 0;
    float mx = 0;
    float my = 0;
    float mz = 0;
    float gyroZ = 0;
    unsigned long last_servo_update = 0;

public:

    SensorManager();

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

    void startSensors(SerialManager &ser, MissionManager &info);

    void writeReleaseServo(int pos);

    void writeGyroServoRight(int pos);

    void writeGyroServoLeft(int pos);

    void writeCameraServo(int pos);

    void getGpsAlt(float *alt);

    void getGpsLat(float *lat);

    void getGpsLong(float *lon);

    void getGpsSats(int *sat);

    void getRtcTime(char time_str[DATA_SIZE]);

    void getGpsTime(char time_str[DATA_SIZE]);

    float getVoltage();

    float getRotRate();

    void setRtcTime(int sec, int minute, int hour);

};

#endif /* SENSOR_MANAGER_H */