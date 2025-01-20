#include <ESP32Servo.h>

// Define servo objects
Servo servo1;
Servo servo2;

// Define servo pins
const int servo1Pin = 36; // Change to your servo pin

// Define servo movement angles
const int minAngle = 0;
const int maxAngle = 180;

// Time delay for movement
const int delayTime = 15;

void setup() {
  
  // Attach servos to their respective pins
  servo1.attach(servo1Pin);

  // Start servos at minimum angle
  servo1.write(minAngle);
  servo2.write(minAngle);

  Serial.begin(115200);
  Serial.println("Servo control started");
}

void loop() {
  // Move servos from minAngle to maxAngle
  for (int angle = minAngle; angle <= maxAngle; angle++) {
    servo1.write(angle);
    servo2.write(angle);
    delay(delayTime);
  }

  // Move servos from maxAngle to minAngle
  for (int angle = maxAngle; angle >= minAngle; angle--) {
    servo1.write(angle);
    servo2.write(angle);
    delay(delayTime);
  }
}
