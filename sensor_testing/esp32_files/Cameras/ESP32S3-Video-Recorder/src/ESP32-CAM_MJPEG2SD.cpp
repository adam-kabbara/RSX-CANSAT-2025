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

// Semaphore to signal pin change activity
SemaphoreHandle_t recordPinActivitySemaphore = NULL;

// ISR to handle pin changes on RECORD_PIN
void IRAM_ATTR handleRecordPinChangeISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (recordPinActivitySemaphore != NULL) {
    xSemaphoreGiveFromISR(recordPinActivitySemaphore, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void setup() {
  pinMode(RECORD_PIN, INPUT_PULLDOWN);
  pinMode(RECORDING_DIAG, OUTPUT);

  // Create the binary semaphore
  recordPinActivitySemaphore = xSemaphoreCreateBinary();
  if (recordPinActivitySemaphore == NULL) {
    LOG_ERR("Failed to create recordPinActivitySemaphore");
    // Handle error appropriately, e.g., by not attaching the interrupt or halting
  } else {
    // Attach the interrupt to RECORD_PIN, triggering on CHANGE (both RISING and FALLING edges)
    attachInterrupt(digitalPinToInterrupt(RECORD_PIN), handleRecordPinChangeISR, CHANGE);
  }

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
  if (recordPinActivitySemaphore != NULL && xSemaphoreTake(recordPinActivitySemaphore, portMAX_DELAY) == pdTRUE) {
    // Interrupt occurred, read the current state of the pin
    int state = digitalRead(RECORD_PIN);
    Serial.print("Interrupt - state: ");
    Serial.println(state);

    if (state == 1) { // Pin is HIGH
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
  // The loop will now block on xSemaphoreTake, so the delay(1000) is no longer needed here for polling.
  // If other periodic tasks were intended for loop(), xSemaphoreTake could use a timeout.
  // For now, this makes the RECORD_PIN handling purely event-driven.

  // vTaskDelete(NULL); // free 8k ram
}

//TODO: low power mode prior to starting recording 
// timestamp for recordings
// cleanup all the bullshit