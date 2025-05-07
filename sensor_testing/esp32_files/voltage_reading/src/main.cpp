#include <HardwareSerial.h>

const int   analogPin = 34;  // Use the correct ADC pin for your ESP32 board
const float R1        = 12000.0; // Updated: 12kΩ (to match 7.4V → 3.3V)
const float R2        = 10000.0; // 10kΩ (kept the same)

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Raw ADC reading [0..4095]
  uint16_t adcRaw = analogRead(analogPin);

  // Calculate voltage at ADC pin
  float adcVoltage = (adcRaw * 3.3) / 4095.0;

  // Calculate actual input voltage (reversed voltage divider formula)
  float inputVoltage = adcVoltage * ((R1 + R2) / R2);

  Serial.print("ADC Reading: ");
  Serial.print(adcRaw);
  Serial.print(" | ADC Voltage: ");
  Serial.print(adcVoltage, 3);
  Serial.print(" V | Input Voltage: ");
  Serial.print(inputVoltage, 3);
  Serial.println(" V");

  delay(1000);
}
