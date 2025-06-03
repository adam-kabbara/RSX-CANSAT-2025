// Daniel Yu
// CANSAT Team 2T4-2T5
// Robotics for Space Exploration
// University of Toronto

// Hall Effect Sensor Code

#include <Arduino.h>

#define HALL_PIN 34     // ADC1_CH6, 12-bit ADC
#define THRESHOLD 2500  // Adjust as needed for your sensor

unsigned long lastPulseTime = 0;
unsigned long pulseInterval = 0;
float RPM = 0.0;
float prevRPM = 0.0;
int lastState = HIGH;

void setup() {
    Serial.begin(115200);
    pinMode(HALL_PIN, INPUT);
    pinMode(2, OUTPUT);
}

void loop() {
    int analogValue = analogRead(HALL_PIN);
    int currState = (analogValue < THRESHOLD) ? LOW : HIGH;

    if (currState == LOW && lastState == HIGH) {
        digitalWrite(2, HIGH);
        unsigned long now = micros();
        pulseInterval = now - lastPulseTime;
        lastPulseTime = now;
        if (pulseInterval > 0) {
            RPM = 60.0 * 1000000.0 / pulseInterval;
            if (RPM > 2000.0) {
                RPM = prevRPM;
            }
            prevRPM = RPM;
        }
    } else {
        digitalWrite(2, LOW);
    }

    Serial.print("Analog: ");
    Serial.print(analogValue);
    Serial.print(" | RPM: ");
    Serial.println(RPM);

    lastState = currState;
    //delay(10);
}
