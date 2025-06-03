#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <unordered_map>
#include <functional>
#include "esp_system.h"
#include <cstring>
#include <stdarg.h>
#include <string>

#define CMD_BUFF_SIZE 128
#define RESP_SIZE 128
#define SEA_LEVEL_PRESSURE_HPA 1013.25
#define WORD_SIZE 64
#define DATA_SIZE 32
#define SENTENCE_SIZE 128
#define DATA_BUFF_SIZE 512
#define TEAM_ID 3114
#define XBEE_BAUD_RATE 57600
#define ALTITUDE_WINDOW_SIZE 40
#define SENSOR_SAMPLE_RATE_HZ 40
#define MAX_LOG_FILE_SIZE_BYTES 125000
#define ADC_LINEAR_REGRESSION 0.119505

#define RX1_PIN 9
#define TX1_PIN 10
#define RX2_PIN 16
#define TX2_PIN 17
#define CAMERA1_SIGNAL_PIN 32
#define CAMERA2_SIGNAL_PIN 33
#define CAMERA1_STATUS_PIN 25
#define CAMERA2_STATUS_PIN 26

#define SERVO_CAMERA_PIN 13
#define SERVO_RELEASE_PIN 12
#define SERVO_GYRO1_PIN 14
#define SERVO_GYRO2_PIN 27

#define HALL_SENSOR_PIN 34
#define ADC_VOLTAGE_PIN 39

#define BNO_CS_PIN 15
#define BNO_INT_PIN 35
#define BNO_RST_PIN 5
#define BNO_SPI_SCK 18
#define BNO_SPI_MISO 19
#define BNO_SPI_MOSI 23

#define RTC_I2C_SDA 21
#define RTC_I2C_SCL 22

#define KALMAN_R 1.0
#define KALMAN_Q 0.01

enum SimModeStatus {
    SIM_OFF = 0,
    SIM_EN = 1,
    SIM_ON = 2
};
enum OperatingState {
    LAUNCH_PAD = 0,
    ASCENT = 1,
    APOGEE = 2,
    DESCENT = 3,
    PROBE_RELEASE = 4,
    LANDED = 5,
    IDLE = 6
};
enum OperatingMode {
    OPMODE_FLIGHT = 0,
    OPMODE_SIM = 1
};

#endif /* GLOBAL_H */