#include <Arduino.h>
#include <Servo.h>

Servo myservo;

#define PI 3.1415926535897932384626433832795

int pos = 0;

void setup(){
    myservo.attach(3);
}

void loop() {
    for (pos = 0; pos <= 180; pos+=1){
        myservo.write(pos);
        delay(5);
    }
    for (pos = 180; pos >= 0; pos-=1){
        myservo.write(pos);
        delay(5);
    }
}