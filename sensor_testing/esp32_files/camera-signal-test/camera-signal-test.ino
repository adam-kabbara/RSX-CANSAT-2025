/**
* simulate a start record command for main camera development
*/

const int GPIO_PIN = 14;

// send a 3 second long high signal on pin 14
void setup() {
  Serial.begin(115200);
  Serial.println("setup started");
  pinMode(GPIO_PIN, OUTPUT);
  digitalWrite(GPIO_PIN, HIGH);
  Serial.println("pin written");
  delay(3000);
  digitalWrite(GPIO_PIN, LOW);
  Serial.println("delay done");
}

void loop() {

}