#include <Adafruit_BNO08x.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// --- BNO08x SPI Pins ---
#define BNO08X_CS 15
#define BNO08X_INT 35
#define BNO08X_RESET 5
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23

SPIClass mySPI(VSPI);
Adafruit_BNO08x bno08x(BNO08X_RESET);
Adafruit_LIS3MDL lis3mdl;

// Kalman filter state
float yaw_estimate = 0.0;
float yaw_cov = 1.0;
const float Q = 0.01;  // process noise
const float R = 1.0;   // measurement noise
unsigned long lastUpdate = 0;

// Convert BNO08x quaternion to yaw (degrees)
float quaternionToYawDegrees(float r, float i, float j, float k) {
  float yaw = atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k));
  float yaw_deg = yaw * 180.0 / PI;
  if (yaw_deg < 0) yaw_deg += 360.0;
  return yaw_deg;
}

// Tilt-compensated heading from magnetometer + accelerometer
float computeTiltCompensatedYaw(float mx, float my, float mz,
                                float ax, float ay, float az) {
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0) return 0;
  ax /= norm;
  ay /= norm;
  az /= norm;

  float pitch = asin(-ax);
  float roll = atan2(ay, az);

  float Xh = mx * cos(pitch) + mz * sin(pitch);
  float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

  float heading = atan2(Yh, Xh);
  float heading_deg = heading * 180.0 / PI;
  if (heading_deg < 0) heading_deg += 360.0;
  return heading_deg;
}

// Kalman filter update
void kalmanUpdate(float gyro_z, float mag_yaw, float dt) {
  yaw_estimate += gyro_z * dt * 180.0 / PI;  // convert rad/s to deg/s
  yaw_cov += Q;

  if (yaw_estimate < 0) yaw_estimate += 360.0;
  if (yaw_estimate >= 360.0) yaw_estimate -= 360.0;

  float K = yaw_cov / (yaw_cov + R);
  yaw_estimate += K * (mag_yaw - yaw_estimate);
  yaw_cov *= (1 - K);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
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
  static float gyroZ = 0;
  static float ax = 0, ay = 0, az = 0;
  float bnoYaw = -1;

  // Timing
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  // Read ONE event (non-blocking)
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;

      case SH2_GYROSCOPE_CALIBRATED:
        gyroZ = sensorValue.un.gyroscope.z;
        break;

      case SH2_ARVR_STABILIZED_RV:
        bnoYaw = quaternionToYawDegrees(
          sensorValue.un.arvrStabilizedRV.real,
          sensorValue.un.arvrStabilizedRV.i,
          sensorValue.un.arvrStabilizedRV.j,
          sensorValue.un.arvrStabilizedRV.k);
        break;
    }
  }

  // Read LIS3MDL magnetometer
  lis3mdl.read();
  float mx = lis3mdl.x;
  float my = lis3mdl.y;
  float mz = lis3mdl.z;

  float magYaw = computeTiltCompensatedYaw(mx, my, mz, ax, ay, az);

  float pitch = asin(-ax);
  float roll = atan2(ay, az);

  // Kalman filter
  kalmanUpdate(gyroZ, magYaw, dt);


  // Compute how much to rotate to face due north (0°)
  float angle_to_north = fmod((360.0 - yaw_estimate), 360.0);

  // Output
  Serial.print("Current Heading (Kalman Yaw): ");
  Serial.print(yaw_estimate, 2);
  //Serial.print(",");
  //Serial.print(pitch, 2);
  //Serial.print(",");
  //Serial.print(roll, 2);
  //Serial.println();
  Serial.print("° | Turn to face North: ");
  Serial.print(angle_to_north, 2);
  Serial.println("°");

  delay(100);
}
