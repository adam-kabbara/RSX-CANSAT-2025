#define NOTE_G  392
#define NOTE_E  330
#define NOTE_D  294
#define NOTE_C  262


int melody[] = {
  NOTE_E, NOTE_D, NOTE_C, NOTE_D,
  NOTE_E, NOTE_E, NOTE_E, NOTE_D,
  NOTE_D, NOTE_D, NOTE_E, NOTE_G,
  NOTE_G, NOTE_E, NOTE_D, NOTE_C,
  NOTE_D, NOTE_E, NOTE_E, NOTE_E,
  NOTE_E, NOTE_D, NOTE_D, NOTE_E,
  NOTE_D, NOTE_C
};

int noteDurations[] = {
  4, 4, 4, 4,
  4, 4, 2, 4,
  4, 2, 4, 4,
  2, 4, 4, 4,
  4, 4, 4, 4,
  4, 4, 4, 4,
  2
};

int speakerPin = 9;

void setup() {
  //pinMode(speakerPin, OUTPUT);

  //for (int thisNote = 0; thisNote < sizeof(melody) / sizeof(int); thisNote++) {
  //  int noteDuration = 1000 / noteDurations[thisNote];
  //  tone(speakerPin, melody[thisNote], noteDuration);

   // int pauseBetweenNotes = noteDuration * 1.30;  // a little pause between notes
   // delay(pauseBetweenNotes);
   // noTone(speakerPin);
  //}
}

void loop() {
  tone(speakerPin, 500,1000);
  // Nothing here, the song will play once when the Arduino is powered on or reset.
}
