// Define the pin to send the signal
const int signalPin = 7; // Replace with your desired pin number

// Define the duration of HIGH and LOW states (in milliseconds)
const int highDuration = 3000; // 500ms HIGH
const int lowDuration = 1000; // 500ms HIGH

int countHigh = 0;
int countLow = 0;


void setup() {
  Serial.begin(115200);
  // Initialize the pin as an output
  pinMode(signalPin, OUTPUT);

}

void loop() {
  if (countHigh < 100) {
    // Set the pin HIGH
    Serial.println("high");
    digitalWrite(signalPin, HIGH);
    countHigh++;
  }

  if (countHigh == 100) {
    if (countLow < 30) {
      // Set the pin LOW
      Serial.println("low");
      digitalWrite(signalPin, LOW);
      countLow++;
    }
  }
  delay(10);
}
