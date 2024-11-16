// Daniel Yu
// CANSAT 2T4-2T5

int hallSensorPin = 2;
int ledPin = 13;

int currState = 0;
int lastState = 1;
float rpm = 0.0;
float prevRPM = 0.0;
unsigned long lastPulseTime = 0;
unsigned long pulseInterval = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(1000);

  pinMode(ledPin, OUTPUT);      
  pinMode(hallSensorPin, INPUT);     
}

void loop() {
  currState = digitalRead(hallSensorPin);

  if (currState == LOW && lastState == HIGH) {
    digitalWrite(ledPin, HIGH);
    unsigned long currentTime = micros();
    pulseInterval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
  } else if (currState == HIGH) {
    digitalWrite(ledPin, LOW);
  }

  rpm = calculateRPM(pulseInterval, prevRPM);
  prevRPM = rpm;

  Serial.print("Current RPM: ");
  Serial.println(rpm);

  lastState = currState;
  delay(10);
}

float calculateRPM(unsigned long pulseInterval, float previous) {
  if ((currState == HIGH) && (pulseInterval > 0)) {
    rpm = (60.0 * 1000000) / pulseInterval;
  }

  if (rpm > 2000.0) {
    rpm = previous;
  }

  return rpm;
}
