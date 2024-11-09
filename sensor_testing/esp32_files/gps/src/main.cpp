#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);

void setup() {
    Serial.begin(115200); // For debugging
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
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
            Serial.print("Satellites: ");
            Serial.println(gps.satellites.value());
            Serial.print("Altitude: ");
            Serial.println(gps.altitude.meters());
            Serial.print("Time: ");
            Serial.println(gps.time.value());
            Serial.println("--------------------------------");
        }
    }
}

// #include <Arduino.h>

// #define RXD2 16
// #define TXD2 17

// #define GPS_BAUD 9600

// // Create an instance of the HardwareSerial class for Serial 2
// HardwareSerial gpsSerial(2);

// void setup(){
//   // Serial Monitor
//   Serial.begin(115200);
  
//   // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
//   gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
//   Serial.println("Serial 2 started at 9600 baud rate");
// }

// void loop(){
//   while (gpsSerial.available() > 0){
//     // get the byte data from the GPS
//     char gpsData = gpsSerial.read();
//     Serial.print(gpsData);
//   }
//   delay(1000);
//   Serial.println("-------------------------------");
// }
