

#include <Arduino.h>
#include <ESP32Servo.h>  // ESP32‐compatible Servo library
#include <Adafruit_BNO08x.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>


// ─── Servo Pin Definitions ───────────────────────────────────────────────────
static const int SERVO_LEFT_PIN  = 2;  // GPIO pin for left fin (adjust as needed)
static const int SERVO_RIGHT_PIN = 4;  // GPIO pin for right fin (adjust as needed)

// ─── Servo Objects ────────────────────────────────────────────────────────────
Servo servoLeft;
Servo servoRight;

SPIClass mySPI(VSPI);
Adafruit_BNO08x bno08x(BNO08X_RESET);
Adafruit_LIS3MDL lis3mdl;

// Remember current servo angles (initialized to center)
float currentAngleLeft = 90.0f;
float currentAngleRight = 90.0f;

void applyFinOutput(float finDelta) {
  // Constrain the incremental change to ±FIN_LIMIT
  if (finDelta >  FIN_LIMIT) finDelta =  FIN_LIMIT;
  if (finDelta < -FIN_LIMIT) finDelta = -FIN_LIMIT;

  // Apply the change incrementally to the current angles
  currentAngleLeft  += finDelta;
  currentAngleRight -= finDelta;

  // Constrain to valid servo range [0°, 180°]
  currentAngleLeft  = constrain(currentAngleLeft,  0.0f, 180.0f);
  currentAngleRight = constrain(currentAngleRight, 0.0f, 180.0f);

  // Send commands to the servos
  servoLeft.write(static_cast<int>(currentAngleLeft));
  servoRight.write(static_cast<int>(currentAngleRight));
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  Serial.println("Starting BNO085 + LIS3MDL Kalman Yaw Fusion");

  // --- BNO08x SPI Init ---
  mySPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &mySPI)) {
    Serial.println(":x: BNO085 not detected.");
    while (1) delay(1000);
  }
  Serial.println(":white_check_mark: BNO08x initialized over SPI.");

  bno08x.enableReport(SH2_ARVR_STABILIZED_RV);
  bno08x.enableReport(SH2_ACCELEROMETER);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);

  // --- LIS3MDL I2C Init ---
  if (!lis3mdl.begin_I2C()) {
    Serial.println(":x: LIS3MDL not detected.");
    while (1) delay(1000);
  }
  Serial.println(":white_check_mark: LIS3MDL initialized over I2C.");

  lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);

  lastUpdate = millis();
}


void loop() {
  
  float finCmd = pidController.update_PID(bno08x, lis3mdl);
  applyFinOutput(finCmd);

  // 5) Wait until next control step
  //    On ESP32 @115K baud, loop runs fast; use a small delay for ~100 Hz update:
  delay(10);
}
