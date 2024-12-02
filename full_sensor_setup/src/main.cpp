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

// Standard atmospheric pressure is 1013.25 hPa --> https://en.wikipedia.org/wiki/Atmospheric_pressure, https://www.noaa.gov/jetstream/atmosphere/air-pressure
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

void hallSensorSetup();
void barometerSetup();
void gpsSetup();

void hallSensorLoop();
void barometerLoop();
void gpsLoop();

float calculateRPM(unsigned long pulseInterval, float previous);
void printData();

// ESP32 pins
int hallSensorPin = 23;
int ledPin = 2;

// Initializing Hall Sensor variables
int currState = 0;
int lastState = 0;
float currRPM = 0.0;
float prevRPM = 0.0;
unsigned long lastPulseTime = 0;
unsigned long pulseInterval = 0;

// Declaring strings to store sensor data
char rpm[10];                                                         // Hall Sensor data
char temp[], pressure[], baroAltitude[], humidity[];                // Barometer data
char time[], latitude[], longitude[], satellites[], gpsAltitude[];  // GPS data

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Setup:");
    
    hallSensorSetup();
    barometerSetup();
    gpsSetup();

    Serial.println("Setup Complete!");
    delay(1000);
    Serial.println("\nGATHERING SENSOR DATA...\n");
}

void loop() {
    hallSensorLoop();
    barometerLoop();
    gpsLoop();
    //printData();
    delay(20); // To accomodate the Hall Sensor sensitivity
}

void hallSensorSetup() {
    Serial.println("Hall Sensor Setup...");
    pinMode(ledPin, OUTPUT);
    pinMode(hallSensorPin, INPUT); 
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
    GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
}

void hallSensorLoop() {
    currState = digitalRead(hallSensorPin);
    // Serial.println(currState); // Debugging statements
    // Serial.println(lastState); // Debugging statements

    // Turns on LED and calculates starting time when magnet comes across the hall effect sensor
    if (currState == LOW && lastState == HIGH) {
        digitalWrite(ledPin, LOW);
        unsigned long currentTime = micros();
        // Serial.println(currentTime); // Debugging statements
        pulseInterval = currentTime - lastPulseTime;
        // Serial.println(pulseInterval); // Debugging statements
        lastPulseTime = currentTime;
    } else if (currState == HIGH) {
        digitalWrite(ledPin, HIGH); // Turns off LED at its falling edge detection
    }

    currRPM = calculateRPM(pulseInterval, prevRPM);
    prevRPM = currRPM; // Resets the RPM state

    Serial.print("Current RPM: ");
    Serial.println(currRPM);

    lastState = currState; // Resets the hall effect state
}

void barometerLoop() { 
    Serial.print("Temperature: ");
    Serial.print(bme.readTemperature());
    Serial.println("°C");

    // Quite accurate according to https://toronto.weatherstats.ca/charts/pressure_station-hourly.html
    // Note: 1 kPa = 10 hPa
    Serial.print("Pressure: ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println("hPa");

    Serial.print("Approx. Altitude: ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println("m");

    Serial.print("Humidity: ");
    Serial.print(bme.readHumidity());
    Serial.println("%");

    Serial.println();
}

void gpsLoop() {
    while(GPS_Serial.available() > 0) {
        char c = GPS_Serial.read();
        gps.encode(c);

        if (gps.location.isUpdated()) {
            // Time
            int hour = gps.time.hour();
            int minute = gps.time.minute();
            int second = gps.time.second();
            Serial.print("Time: ");
            if (hour < 10) {
                Serial.print("0");
            }
            Serial.print(hour);
            Serial.print(":");
            if (minute < 10) {
                Serial.print("0");
            }
            Serial.print(minute);
            Serial.print(":");
            if (second < 10) {
                Serial.print("0");
            }
            Serial.print(second);
            Serial.println(" (UTC Time)");

            // Latitude
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            if (gps.location.lat() > 0) {
                Serial.println("°N");
            } else {
                Serial.println("°S");
            }
            
            // Longitude
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            if (gps.location.lng() > 0) {
                Serial.println("°E");
            } else {
                Serial.println("°W");
            }

            // Satellites
            Serial.print("Satellites: ");
            Serial.println(gps.satellites.value());
            
            // Altitude 
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println("m");
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

    rpm[] = currRPM;
    return currRPM;
}

void printData() {
    Serial.println("---------- Satellite Data ----------");

    Serial.println("------------------------------------");
}
