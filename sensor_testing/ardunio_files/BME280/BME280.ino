#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Create an instance of the sensor
Adafruit_BME280 bme;

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for serial port to be available

    // Initialize the sensor
    if (!bme.begin(0x76) && !bme.begin(0x77)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring or SDO connection!");
        while (1);
    }
}

void loop() {
    // Reading the sensor values
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
    float seaLevelPressure = 1013.25; // hPa
    float altitude = bme.readAltitude(seaLevelPressure);

    // Print the values to the Serial Monitor
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Altitude = ");
    Serial.print(altitude);
    Serial.println(" meters");

    // Wait for 2 seconds before the next loop
    delay(2000);
}
