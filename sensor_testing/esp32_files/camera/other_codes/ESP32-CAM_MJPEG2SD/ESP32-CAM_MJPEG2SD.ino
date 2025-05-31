/*
* Capture ESP32 Cam JPEG images into a AVI file and store on SD
* AVI files stored on the SD card can also be selected and streamed to a browser as MJPEG.
*
* s60sc 2020 - 2024
*/

#include "appGlobals.h"

bool printInitMessage = true;
#define GPIO_PIN 14
extern bool forceRecord;

void setup() {
  pinMode(GPIO_PIN, INPUT_PULLUP);

  logSetup();
  // prep storage
  if (startStorage()) {
    // Load saved user configuration
    if (loadConfig()) {
      // initialise camera
      if (psramFound()) {
        LOG_INF("PSRAM size: %s", fmtSize(ESP.getPsramSize()));
        if (ESP.getPsramSize() > 3 * ONEMEG) prepCam();
        else snprintf(startupFailure, SF_LEN, STARTUP_FAIL "Insufficient PSRAM for app: %s", fmtSize(ESP.getPsramSize()));
      } else snprintf(startupFailure, SF_LEN, STARTUP_FAIL "Need PSRAM to be enabled");
    }
  }

  if (strlen(startupFailure)) LOG_WRN("%s", startupFailure);
  else {
    startSustainTasks();
    prepRecording();
    checkMemory();
  }
}

void loop() {
  if (printInitMessage) {
    // confirm not blocked in setup
    LOG_INF("=============== Total tasks: %u ===============\n", uxTaskGetNumberOfTasks() - 1);
    delay(1000);
    printInitMessage = false;
  }

  int state = digitalRead(GPIO_PIN);
  Serial.println(state);
  if (state == HIGH) {
    if (!forceRecord)
      forceRecord = true;
    else
      forceRecord = false;
  }
  delay(3000);

  // vTaskDelete(NULL); // free 8k ram
}
