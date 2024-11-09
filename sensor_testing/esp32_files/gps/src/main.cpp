#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

void setup() {
    Serial.begin(115200); // For monitoring of the GPS data
    GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // Set baud rate and pins for GPS

    Serial.println("GPS Module is starting up...");
}

void loop() {
    while (GPS_Serial.available() > 0) {
        char c = GPS_Serial.read();
        // Serial.write(c); // Print raw data for debugging
        gps.encode(c); // Feed the GPS parser

        if (gps.location.isUpdated()) {
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            if (gps.location.lat() > 0) {
                Serial.println(" °N");
            } else {
                Serial.println(" °S");
            }
            
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            if (gps.location.lng() > 0) {
                Serial.println(" °E");
            } else {
                Serial.println(" °W");
            }
            
            Serial.print("Satellites: ");
            Serial.println(gps.satellites.value());
            
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");

            Serial.print("Speed: ");
            Serial.print(gps.speed.knots());
            Serial.print(" knots ");
            Serial.print("(");
            Serial.print(gps.speed.mps());
            Serial.println(" m/s)");

            Serial.print("Time: ");
            Serial.println(gps.time.value());

            Serial.println("--------------------------------");
        }
    }
}
