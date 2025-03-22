// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// Full Sensor Setup

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ESP32 pins
#define HALL_SENSOR_PIN 23
#define ONBOARD_LED_PIN 2
#define ESP32_RX 9
#define ESP32_TX 10

// Standard atmospheric pressure is 1013.25 hPa --> https://en.wikipedia.org/wiki/Atmospheric_pressure, https://www.noaa.gov/jetstream/atmosphere/air-pressure
#define SEALEVELPRESSURE_HPA (1013.25)

void hallSensorSetup();
void barometerSetup();
void gpsSetup();

void hallSensorLoop();
void barometerLoop();
void gpsLoop();

float calculateRPM(unsigned long pulseInterval, float previous);
void serialPrintData();

// Initializing hall sensor variables
int currState = 0;
int lastState = 0;
float currRPM = 0.0;
float prevRPM = 0.0;
unsigned long lastPulseTime = 0;
unsigned long pulseInterval = 0;

// Defining a struct to store sensor data
struct SensorData {
    char rpm[15];                                                                       // Hall Sensor data
    char temp[15], pressure[15], baroAltitude[15], humidity[15];                        // Barometer data
    char utcTime[20], latitude[20], longitude[20], satellites[20], gpsAltitude[20];     // GPS data
};

Adafruit_BME280 bme;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
SensorData sensorData;

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting Setup:");
    
    hallSensorSetup();
    barometerSetup();
    gpsSetup();

    Serial.println("Setup Complete!");
    delay(1000);
    Serial.println("\nGATHERING SENSOR DATA...");
}

void loop() {
    hallSensorLoop();
    barometerLoop();
    gpsLoop();
    serialPrintData();
    delay(20); // To accomodate the Hall Sensor sensitivity
}

void hallSensorSetup() {
    Serial.println("Hall Sensor Setup...");
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    pinMode(HALL_SENSOR_PIN, INPUT);
}

void barometerSetup() {
    Serial.println("Barometer Setup...");

    // Checking if the I2C connection is setup correctly 
    bool status;
    status = bme.begin(0x77);  
    if (!status) {
        Serial.println("Could not find a valid barometer sensor, check wiring!");
        while (1);
    }
}

void gpsSetup() {
    Serial.println("GPS Setup...");
    GPS_Serial.begin(9600, SERIAL_8N1, ESP32_RX, ESP32_TX);

    // Initializing GPS values before valid connections
    // strcpy(sensorData.utcTime, "n/a");
    // strcpy(sensorData.latitude, "n/a");
    // strcpy(sensorData.longitude, "n/a");
    // strcpy(sensorData.satellites, "n/a");
    // strcpy(sensorData.gpsAltitude, "n/a");
}

void hallSensorLoop() {
    currState = digitalRead(HALL_SENSOR_PIN);

    // Turns on LED and calculates starting time when magnet comes across the hall effect sensor
    if (currState == LOW && lastState == HIGH) {
        digitalWrite(ONBOARD_LED_PIN, LOW);
        unsigned long currentTime = micros();
        pulseInterval = currentTime - lastPulseTime;
        lastPulseTime = currentTime;
    } else if (currState == HIGH) {
        digitalWrite(ONBOARD_LED_PIN, HIGH); // Turns off LED at its falling edge detection
    }

    currRPM = calculateRPM(pulseInterval, prevRPM);
    prevRPM = currRPM; // Resets the RPM state

    lastState = currState; // Resets the hall effect state
}

void barometerLoop() { 
    snprintf(sensorData.temp, sizeof(sensorData.temp), "%.2f°C", bme.readTemperature());
    snprintf(sensorData.pressure, sizeof(sensorData.pressure), "%.2f hPa", (bme.readPressure() / 100.0F));
    snprintf(sensorData.baroAltitude, sizeof(sensorData.baroAltitude), "%.2f m", bme.readAltitude(SEALEVELPRESSURE_HPA));
    snprintf(sensorData.humidity, sizeof(sensorData.humidity), "%.2f%%", bme.readHumidity());
}

void gpsLoop() {
    while(GPS_Serial.available() > 0) {
        char c = GPS_Serial.read();
        gps.encode(c);

        if (gps.location.isUpdated() && gps.location.isValid()) {
            snprintf(sensorData.utcTime, sizeof(sensorData.utcTime), "%02d:%02d:%02d (UTC Time)", gps.time.hour(), gps.time.minute(), gps.time.second());
            snprintf(sensorData.latitude, sizeof(sensorData.latitude), "%.6f%s°", gps.location.lat(), gps.location.lat() > 0 ? "N" : "S");
            snprintf(sensorData.longitude, sizeof(sensorData.longitude), "%.6f%s°", gps.location.lng(), gps.location.lng() > 0 ? "E" : "W");
            snprintf(sensorData.satellites, sizeof(sensorData.satellites), "%d", gps.satellites.value());
            snprintf(sensorData.gpsAltitude, sizeof(sensorData.gpsAltitude), "%.2f m", gps.altitude.meters());
        }
    }
}

// Calculates the RPM value using the subtracted time
float calculateRPM(unsigned long pulseInterval, float previous) {
    if ((currState == LOW) && (lastState == HIGH) && (pulseInterval > 0)) {
        currRPM = (60.0 * 1000000) / pulseInterval;
    }

    // Filters out extreme RPM calculations 
    if (currRPM > 2000.0) {
        currRPM = previous;
    }

    snprintf(sensorData.rpm, sizeof(sensorData.rpm), "%.2f", currRPM);
    return currRPM;
}

void serialPrintData() {
    Serial.println("\n---------- Sensor Data ----------");
    
    Serial.print("RPM: ");
    Serial.println(sensorData.rpm);

    Serial.print("Temperature: ");
    Serial.println(sensorData.temp);

    Serial.print("Pressure: ");
    Serial.println(sensorData.pressure);

    Serial.print("Barometer Altitude: ");
    Serial.println(sensorData.baroAltitude);

    Serial.print("Humidity: ");
    Serial.println(sensorData.humidity);

    Serial.print("UTC Time: ");
    Serial.println(sensorData.utcTime);

    Serial.print("Latitude: ");
    Serial.println(sensorData.latitude);

    Serial.print("Longitude: ");
    Serial.println(sensorData.longitude);

    Serial.print("Connected Satellites: ");
    Serial.println(sensorData.satellites);

    Serial.print("GPS Altitude: ");
    Serial.println(sensorData.gpsAltitude);

    Serial.println("---------------------------------\n");
}
