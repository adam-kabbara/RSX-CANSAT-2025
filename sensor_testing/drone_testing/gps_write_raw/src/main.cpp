// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// GPS High Altitude Testing --> 4AM GPS Testing for CanSat Virginia 2025 lmao
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
    Serial.println("Starting GPS Module...");
    delay(1000); // Allow time for Serial to initialize
    
    // RX0 and TX0 pins for GPS (PCB Version)
    // GPS_Serial.begin(9600, SERIAL_8N1, 3, 1); // Set baud rate and pins for GPS

    // RX1 and TX1 pins for GPS (Breadboard Version)
    GPS_Serial.begin(9600, SERIAL_8N1, 9, 10); // Set baud rate and pins for GPS

    // Initialize SPIFFS
    // if (!SPIFFS.begin(true)) {
    //     Serial.println("Failed to mount file system");
    //     return;
    // }

    Serial.println("GPS Module started successfully!");
}

void loop() {
    // Open or create the gps_data.txt file for appending
    // gpsFile = SPIFFS.open("/gps_data.txt", FILE_APPEND);
    // if (!gpsFile) {
    //     Serial.println("Failed to open gps_data.txt for writing");
    //     return;
    // }

    // Process incoming GPS data
    // while (GPS_Serial.available() > 0) {
    //     char c = GPS_Serial.read();
    //     //gpsFile.write(c);      // Write raw data to the file
    //     Serial.write(c);       // Print raw data to Serial Monitor
    //     gps.encode(c);         // Feed data to TinyGPS++ for parsing
    // }

    // int gps_avail = GPS_Serial.available();
    // for(int i = 0; i < gps_avail; i++)
    // {
    //     //Serial.print(GPS_Serial.read());
    //     //Serial.print(GPS_Serial.readStringUntil('\n'));
    //     Serial.print((char)GPS_Serial.read());
    //     gps.encode(GPS_Serial.read());
    // }

    // Get the number of available bytes
    int gps_avail = GPS_Serial.available();
    String gpsData = "";
    for (int i = 0; i < gps_avail; i++) {
        gpsData += (char)GPS_Serial.read();
    }

    // Print the full burst to Serial
    if (gpsData.length() > 0) {
        Serial.print("RAW GPS BURST: \n");
        Serial.print(gpsData);
        // Feed each character to TinyGPS++
        for (size_t i = 0; i < gpsData.length(); i++) {
            gps.encode(gpsData[i]);
        }
    }

    // Print parsed GPS info if available
    if (gps.location.isValid() && gps.location.isUpdated()) {
        Serial.print("\n\nGPS DATA:\n");
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        Serial.print("Altitude: ");
        Serial.println(gps.altitude.meters());
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("Time (UTC): ");
        if (gps.time.isValid()) {
            char timeStr[16];
            sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
            Serial.println(timeStr);
        } else {
            Serial.println("INVALID");

        Serial.println();
        }
    }

    //gpsFile.close(); // Close the file to save after changes

    delay(2000);
}
