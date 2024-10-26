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
float prev_rpm = 0.0;

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

  calculateRPM(counter, prev_rpm);
  prev_rpm = rpm;

  Serial.print("Current RPM: ");
  Serial.println(rpm);

  lastState = currState;
  delay(10);
}

float calculateRPM(int revolution, float previous) {
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

  if (rpm > 500.0) {
    rpm = previous;
  }

  return rpm;
}
