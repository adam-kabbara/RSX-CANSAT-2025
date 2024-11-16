// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// GPS High Altitude Testing
// Reading and deleting from ESP32

#include "FS.h"
#include "SPIFFS.h"

File gpsFile;

void setup() {
    Serial.begin(115200);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount file system");
        return;
    }

    // Open the file in read mode to read its contents
    gpsFile = SPIFFS.open("/gps_data.txt", FILE_READ);
    if (!gpsFile) {
        Serial.println("Failed to open gps_data.txt for reading");
        return;
    }

    // Read and print the contents of the file
    Serial.println("Opening gps_data.txt:");
    while (gpsFile.available()) {
        Serial.write(gpsFile.read());  // Read a byte and print it to the Serial Monitor
    }

    // Close the file after reading
    gpsFile.close();

    Serial.println("\nFile reading complete.");
    Serial.println("Type 'delete' in the Serial Monitor to delete gps_data.txt.");
}

void loop() {
    // Check if data is available in the Serial Monitor
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // Read the input command
        command.trim(); // Remove leading and trailing whitespace

        if (command.equalsIgnoreCase("delete")) {
            if (SPIFFS.remove("/gps_data.txt")) {
                Serial.println("gps_data.txt deleted successfully.");
            } else {
                Serial.println("Failed to delete gps_data.txt. File may not exist.");
            }
        } else {
            Serial.println("Unknown command. Type 'delete' to delete gps_data.txt.");
        }
    }
}
