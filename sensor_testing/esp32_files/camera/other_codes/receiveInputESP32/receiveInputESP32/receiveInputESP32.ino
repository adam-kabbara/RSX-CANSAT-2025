#define GPIO_PIN 21

void setup() {
  pinMode(GPIO_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  int state = digitalRead(GPIO_PIN);
  Serial.println(state);
  if (state == HIGH) {
    Serial.println("Signal detected: HIGH");
  } else {
    Serial.println("Signal detected: LOW");
  }
  delay(100);

}
