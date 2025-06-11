// Date and time functions using a DS1307 RTC connected via I2C and Wire lib

#include <Arduino.h>
#include <uRTCLib.h>

// Define I2C pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

uRTCLib rtc(0x68);

// Function to set only hours and minutes
void setTimeOnly(int hour, int minute) {
	// Set time with other values at 0
	// Parameters: second, minute, hour, dayOfWeek, dayOfMonth, month, year
	rtc.set(0, minute, hour, 1, 1, 1, 0);
}

void setup() {
	delay(2000);
	Serial.begin(115200);
	Serial.println("Serial OK");

	// Initialize I2C with explicit pins for ESP32
	Wire.begin(I2C_SDA, I2C_SCL);
	
	// Check if RTC is responding
	Wire.beginTransmission(0x68);
	byte error = Wire.endTransmission();
	
	if (error == 0) {
		Serial.println("RTC found!");
		// Example: Set time to 14:30 (2:30 PM)
		setTimeOnly(14, 30);
	} else {
		Serial.print("RTC not found! Error: ");
		Serial.println(error);
	}
}

void loop() {
	rtc.refresh();

	Serial.print("RTC DateTime: ");
	Serial.print(rtc.year());
	Serial.print('/');
	Serial.print(rtc.month());
	Serial.print('/');
	Serial.print(rtc.day());

	Serial.print(' ');

	Serial.print(rtc.hour());
	Serial.print(':');
	Serial.print(rtc.minute());
	Serial.print(':');
	Serial.print(rtc.second());

	Serial.print(" DOW: ");
	Serial.print(rtc.dayOfWeek());

	Serial.print(" - Temp: ");
	Serial.print(rtc.temp()  / 100);

	Serial.println();

	delay(1000);
}