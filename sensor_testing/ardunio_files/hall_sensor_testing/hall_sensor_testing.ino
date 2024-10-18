// Daniel Yu
// CANSAT 2T4-2T5

int hallSensorPin = 2;     
int ledPin =  13;    
int state = 0;          

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(1000);

  pinMode(ledPin, OUTPUT);      
  pinMode(hallSensorPin, INPUT);     
}

void loop(){
  state = digitalRead(hallSensorPin);

  if (state == LOW) { 
    digitalWrite(ledPin, HIGH);
    Serial.println("Detected Magnet: LED ON!");
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("Lost Magnet: LED OFF!");
  }
}