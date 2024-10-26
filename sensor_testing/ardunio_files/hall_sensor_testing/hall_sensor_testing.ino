// Daniel Yu
// CANSAT 2T4-2T5

int hallSensorPin = 2;     
int ledPin =  13;
int currState = 0;
int lastState = 1;
int counter = 0; 
float onTime = 0.0;
float offTime = 0.0;
float rpm = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(1000);

  pinMode(ledPin, OUTPUT);      
  pinMode(hallSensorPin, INPUT);     
}

void loop(){
  currState = digitalRead(hallSensorPin);

  if (currState == LOW) {
    digitalWrite(ledPin, HIGH);
    if (lastState != currState) {
      // Serial.println("Detected Magnet: LED ON!");
      // Serial.print("At Time: ");
      // onTime = counter * 0.01;
      // Serial.print(onTime);
      // Serial.println(" seconds.");
      counter = 0;
    } else {
      counter++;
    }
  } else {
    digitalWrite(ledPin, LOW);
    if (lastState != currState) {
      // Serial.println("Lost Magnet: LED OFF!");
      // Serial.print("At Time: ");
      // offTime = counter * 0.01;
      // Serial.print(offTime);
      // Serial.println(" seconds.");
    }
  }

  calculateRPM(counter);
  Serial.print("Current RPM: ");
  Serial.println(rpm);

  lastState = currState;
  delay(10);
}

float calculateRPM(int revolution) {
  // float oneRotationTime = end - start;
  // Serial.println(end);
  // Serial.println(start);
  // Serial.println(oneRotationTime);
  float oneRotationTime = 0.0;

  if (lastState != currState) {
    oneRotationTime = revolution * 0.01;
    // Serial.println(oneRotationTime);
  }
  
  if (oneRotationTime > 0) {
    rpm = (60.0 / oneRotationTime);
  }

  return rpm;
}

// // Daniel Yu
// // CANSAT 2T4-2T5

// int hallSensorPin = A0;     
// int ledPin =  13;
// int currState = 0;
// int lastState = 1;
// int counter = 0; 
// float rpm = 0.0;

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Starting...");
//   delay(1000);

//   pinMode(ledPin, OUTPUT);      
//   pinMode(hallSensorPin, INPUT);     
// }

// void loop(){
//   currState = analogRead(hallSensorPin);

//   if (currState <= 23) {
//     digitalWrite(ledPin, HIGH);
//     if (lastState <= 23) {
//       // DEBUGGING PRINT STATEMENTS
//       // Serial.println("Detected Magnet: LED ON!");
//       counter = 0;
//     } else {
//       counter++;
//     }
//   } else {
//     digitalWrite(ledPin, LOW);
//     // DEBUGGING PRINT STATEMENTS
//     // if (lastState != currState) {
//     //   // Serial.println("Lost Magnet: LED OFF!");
//     // }
//   }

//   calculateRPM(counter);
//   // Serial.print("Current RPM: ");
//   // Serial.println(rpm);

//   Serial.println(currState);

//   lastState = currState;
//   // delay(10);
// }

// float calculateRPM(int revolution) {
//   float oneRotationTime = 0.0;

//   if (currState > 23 & lastState <= 23) {
//     oneRotationTime = revolution * 0.01;
//     // DEBUGGING PRINT STATEMENTS
//     // Serial.println(oneRotationTime);
//   }
  
//   if (oneRotationTime > 0) {
//     rpm = (60.0 / oneRotationTime);
//   }

//   return rpm;
// }
