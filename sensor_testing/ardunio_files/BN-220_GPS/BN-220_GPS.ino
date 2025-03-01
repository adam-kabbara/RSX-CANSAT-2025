#include <SoftwareSerial.h>

// Define RX and TX pins for GPS
#define GPS_RX 10
#define GPS_TX 11

// Initialize SoftwareSerial for GPS communication
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

void setup() {
  // Start the hardware Serial Monitor
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial Monitor to open
  }

  // Start the GPS Serial
  gpsSerial.begin(9600);
  Serial.println("BN-220 GPS Module: Raw Data Display");
}

void loop() {
  // Check if data is available from the GPS module
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read(); // Read each byte from the GPS module
    Serial.write(c);           // Print the byte to the Serial Monitor
  }
}
