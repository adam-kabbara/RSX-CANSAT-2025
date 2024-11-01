#include <Arduino.h>

// ESP32 Pins
int hallSensorPin = 23;
int ledPin = 2;

// Initializing variables
int currState = 0;
int lastState = 1;
float rpm = 0.0;
float prevRPM = 0.0;
unsigned long lastPulseTime = 0;
unsigned long pulseInterval = 0;

float calculateRPM(unsigned long pulseInterval, float previous);

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");
    delay(1000);

    pinMode(ledPin, OUTPUT);      
    pinMode(hallSensorPin, INPUT); 
}

void loop() {
    currState = digitalRead(hallSensorPin);

    // Turns on LED and calculates starting time when magnet comes across the hall effect sensor
    if (currState == LOW && lastState == HIGH) {
        digitalWrite(ledPin, HIGH);
        unsigned long currentTime = micros();
        pulseInterval = currentTime - lastPulseTime;
        lastPulseTime = currentTime;
    } else if (currState == HIGH) {
        digitalWrite(ledPin, LOW); // Turns off LED at its falling edge
    }

    rpm = calculateRPM(pulseInterval, prevRPM);
    prevRPM = rpm; // Resets the RPM state

    Serial.print("Current RPM: ");
    Serial.println(rpm);

    lastState = currState; // Resets the hall effect state
    delay(10);
}

// Calculates the RPM value using the subtracted time
float calculateRPM(unsigned long pulseInterval, float previous) {
    if ((currState == HIGH) && (pulseInterval > 0)) {
        rpm = (60.0 * 1000000) / pulseInterval;
    }

    // Filters out extreme RPM calculations 
    if (rpm > 2000.0) {
        rpm = previous;
    }

    return rpm;
}
