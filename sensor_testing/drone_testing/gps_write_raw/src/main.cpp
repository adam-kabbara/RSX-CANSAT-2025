// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// GPS High Altitude Testing
// Writing to ESP32

#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "FS.h"
#include "SPIFFS.h"

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
File gpsFile;

void setup() {
    Serial.begin(115200); // For monitoring
    GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // Set baud rate and pins for GPS

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount file system");
        return;
    }

    Serial.println("GPS Module is starting up...");
}

void loop() {
    // Open or create the gps_data.txt file for appending
    gpsFile = SPIFFS.open("/gps_data.txt", FILE_APPEND);
    if (!gpsFile) {
        Serial.println("Failed to open gps_data.txt for writing");
        return;
    }

    // Process incoming GPS data
    while (GPS_Serial.available() > 0) {
        char c = GPS_Serial.read();
        gpsFile.write(c);      // Write raw data to the file
        Serial.write(c);       // Print raw data to Serial Monitor
        gps.encode(c);         // Feed data to TinyGPS++ for parsing
    }

    gpsFile.close(); // Close the file to save after changes

    delay(2000);
}
