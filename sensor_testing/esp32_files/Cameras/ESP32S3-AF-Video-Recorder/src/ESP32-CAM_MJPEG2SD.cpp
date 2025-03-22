/*
 * Capture ESP32 Cam JPEG images into a AVI file and store on SD
 * AVI files stored on the SD card can also be selected and streamed to a
 * browser as MJPEG.
 *
 * s60sc 2020 - 2024
 */
#include <Arduino.h>
#include "appGlobals.h"
#include "ESP32_OV5640_AF.h"


bool printInitMessage = true;
#define GPIO_PIN 14
extern bool forceRecord;

OV5640 ov5640 = OV5640();

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

  sensor_t *sensor = esp_camera_sensor_get();
  ov5640.start(sensor);

  // Download Firmware for AF
  if (ov5640.focusInit() == 0) {
    Serial.println("OV5640_Focus_Init Successful!");
  }

  // Set Continuous AF MODE
  if (ov5640.autoFocusMode() == 0) {
    Serial.println("OV5640_Auto_Focus Successful!");
  }
}

void loop() {
  if (printInitMessage) {
    // confirm not blocked in setup
    LOG_INF("=============== Total tasks: %u ===============\n",
            uxTaskGetNumberOfTasks() - 1);
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

  uint8_t rc = ov5640.getFWStatus();
  Serial.printf("FW_STATUS = 0x%x\n", rc);
  if (rc == -1) {
    Serial.println("Check your OV5640");
  } else if (rc == FW_STATUS_S_FOCUSED) {
    Serial.println("YAY Focused!");
    // esp_err_t err = esp_camera_deinit();
    // if (err != ESP_OK) {
    //   Serial.printf("Camera de-init failed with error 0x%x", err);
    // }
    // esp_deep_sleep_start();
  } else if (rc == FW_STATUS_S_FOCUSING) {
    Serial.println("STILL FOCUSING!");
  } else {
    Serial.println("IDK");
  }

  delay(3000);

  // vTaskDelete(NULL); // free 8k ram
}
