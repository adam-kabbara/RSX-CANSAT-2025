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
        gps.encode(c);
        
        // Log parsed GPS data to the file
        gpsFile.println("\n------- Parsed GPS Data -------");
        gpsFile.printf("Date: %02d/%02d/%d\n", gps.date.day(), gps.date.month(), gps.date.year());
        gpsFile.printf("Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
        gpsFile.printf("Latitude: %.6f %s\n", gps.location.lat(), gps.location.lat() > 0 ? "N" : "S");
        gpsFile.printf("Longitude: %.6f %s\n", gps.location.lng(), gps.location.lng() > 0 ? "E" : "W");
        gpsFile.printf("Satellites: %d\n", gps.satellites.value());
        gpsFile.printf("Altitude: %.2f m\n", gps.altitude.meters());
        gpsFile.printf("Speed: %.2f knots (%.2f m/s)\n", gps.speed.knots(), gps.speed.mps());
        gpsFile.println("--------------------------------");

        Serial.println("\n------- Parsed GPS Data -------");
        Serial.printf("Date: %02d/%02d/%d\n", gps.date.day(), gps.date.month(), gps.date.year());
        Serial.printf("Time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
        Serial.printf("Latitude: %.6f %s\n", gps.location.lat(), gps.location.lat() > 0 ? "N" : "S");
        Serial.printf("Longitude: %.6f %s\n", gps.location.lng(), gps.location.lng() > 0 ? "E" : "W");
        Serial.printf("Satellites: %d\n", gps.satellites.value());
        Serial.printf("Altitude: %.2f m\n", gps.altitude.meters());
        Serial.printf("Speed: %.2f knots (%.2f m/s)\n", gps.speed.knots(), gps.speed.mps());
        Serial.println("--------------------------------");

        delay(1000);
    }

    gpsFile.close(); // Close the file to save after changes

    delay(2000);
}
