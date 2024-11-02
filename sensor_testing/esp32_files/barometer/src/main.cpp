// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// Barometer Code

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Standard atmospheric pressure is 1013.25 hPa --> https://en.wikipedia.org/wiki/Atmospheric_pressure, https://www.noaa.gov/jetstream/atmosphere/air-pressure
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

void printValues();

void setup() {
    Serial.begin(9600);
    Serial.println(F("\nBooting up BME280 testing..."));

    // Checking if the I2C connection is setup correctly 
    bool status;
    status = bme.begin(0x77);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    Serial.println("\n----- Starting Test -----\n");
}

void loop() { 
    printValues();
    delay(2000);
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    // Convert temperature to Fahrenheit
    /*Serial.print("Temperature = ");
    Serial.print(1.8 * bme.readTemperature() + 32);
    Serial.println(" *F");*/

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}
