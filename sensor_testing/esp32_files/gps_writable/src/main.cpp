// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// GPS High Altitude Testing

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
    
    // Open or create the gps_data.txt file for writing
    gpsFile = SPIFFS.open("/gps_data.txt", FILE_WRITE);
    if (!gpsFile) {
        Serial.println("Failed to open gps_data.txt for writing");
        return;
    }

    // Write data to the file
    gpsFile.println("Hello from ESP32!");
    gpsFile.close();
    Serial.println("Data written to file");
}

void loop() {
    // Open the file in read mode to read its contents
    gpsFile = SPIFFS.open("/gps_data.txt", FILE_READ);
    if (!gpsFile) {
        Serial.println("Failed to open gps_data.txt for reading");
        return;
    }

    // Read and print the contents of the file
    Serial.println("File contents:");
    while (gpsFile.available()) {
        Serial.write(gpsFile.read());  // Read a byte and print it to the Serial Monitor
    }

    // Close the file after reading
    gpsFile.close();

    delay(5000);
}
