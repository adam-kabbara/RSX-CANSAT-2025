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
            // Latitude location
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            if (gps.location.lat() > 0) {
                Serial.println(" °N");
            } else {
                Serial.println(" °S");
            }
            
            // Longitude location
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            if (gps.location.lng() > 0) {
                Serial.println(" °E");
            } else {
                Serial.println(" °W");
            }
            
            // Number of satellites
            Serial.print("Satellites: ");
            Serial.println(gps.satellites.value());
            
            // Altitude 
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" m");

            // Speed in knots and m/s
            Serial.print("Speed: ");
            Serial.print(gps.speed.knots());
            Serial.print(" knots ");
            Serial.print("(");
            Serial.print(gps.speed.mps());
            Serial.println(" m/s)");

            // Prints the current time in HH:MM:SS format with 24-hour clock
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
            Serial.println(second);

            // Prints the current date in DD/MM/YYYY format
            int day = gps.date.day();
            int month = gps.date.month();
            int year = gps.date.year();
            Serial.print("Date: ");
            if (day < 10) {
                Serial.print("0");
            }
            Serial.print(day);
            Serial.print("/");
            if (month < 10) {
                Serial.print("0");
            }
            Serial.print(month);
            Serial.print("/");
            Serial.println(year);

            Serial.println("--------------------------------");
        }
    }
}
