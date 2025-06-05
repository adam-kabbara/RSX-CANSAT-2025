/*
 * Capture ESP32 Cam JPEG images into a AVI file and store on SD
 * AVI files stored on the SD card can also be selected and streamed to a
 * browser as MJPEG.
 *
 * s60sc 2020 - 2024
 */
#include "appGlobals.h"
#include <Arduino.h>

bool printInitMessage = true;
#define RECORD_PIN 14
extern bool forceRecord;
bool stopRecording = false;
bool startRecording = true;

volatile uint8_t pulse_detect = 0;
volatile uint8_t pin_changed = 0;

// ISR to handle pin changes on RECORD_PIN
void IRAM_ATTR ISR_pin_change() 
{
  if (digitalRead(RECORD_PIN)) {
    pulse_detect = 1;
  } else {
    pulse_detect = 0;
  }
  pin_changed = 1;
}

void setup() {
  pinMode(RECORD_PIN, INPUT_PULLDOWN);
  pinMode(RECORDING_DIAG, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RECORD_PIN), ISR_pin_change, CHANGE);

  logSetup();
  // prep storage
  if (startStorage()) {
    // Load saved user configuration
    if (loadConfig()) {
      // initialise camera
      if (psramFound()) {
        LOG_INF("PSRAM size: %s", fmtSize(ESP.getPsramSize()));
        if (ESP.getPsramSize() > 3 * ONEMEG)
          prepCam();
        else
          snprintf(startupFailure, SF_LEN,
                   STARTUP_FAIL "Insufficient PSRAM for app: %s",
                   fmtSize(ESP.getPsramSize()));
      } else
        snprintf(startupFailure, SF_LEN,
                 STARTUP_FAIL "Need PSRAM to be enabled");
    }
  }

  if (strlen(startupFailure))
    LOG_WRN("%s", startupFailure);
  else {
    prepRecording();
    checkMemory();
  }
}

void loop() {
  if (printInitMessage) {
    // confirm not blocked in setup
    LOG_INF("=============== Total tasks: %u ===============\n",
            uxTaskGetNumberOfTasks() - 1);
    // delay(1000);
    printInitMessage = false;
  }

  // Wait for the semaphore to be given by the ISR
  if (pin_changed) {
    pin_changed = 0;
    Serial.print("Interrupt - state: ");
    Serial.println(pulse_detect);

    if (pulse_detect == 1) { // Pin is HIGH
      forceRecord = forceRecord ? !stopRecording : startRecording;

      if (!forceRecord && stopRecording) {
        startRecording = false;
      }
      Serial.print("bool forceRecord: ");
      Serial.println(forceRecord);
    } else { // Pin is LOW (state == 0)
      if (forceRecord) {
        stopRecording = true;
      } else {
        stopRecording = false;
      }

      if (!startRecording) {
        startRecording = true;
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(25));

  // vTaskDelete(NULL); // free 8k ram
}

//TODO: low power mode prior to starting recording 
// timestamp for recordings
// cleanup all the bullshit
