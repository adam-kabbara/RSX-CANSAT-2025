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

// --------------------------
// Simple PID + Stability Monitor
//       for ESP32 (Arduino)
// --------------------------

// ===== PID Controller Class =====
class PID {
public:
  PID(float Kp_, float Ki_, float Kd_, float output_limit_)
    : Kp(Kp_), Ki(Ki_), Kd(Kd_), output_limit(output_limit_),
      integral(0.0f), prev_error(0.0f), prev_time(0)
  {}

  // Call in loop(); returns a fin‐deflection command in degrees (±output_limit)
  float compute(float setpoint_deg, float current_deg) {
    unsigned long now = millis();
    float dt = (prev_time == 0) ? 0.0f : float(now - prev_time) / 1000.0f;
    prev_time = now;

    // Compute angular error in [–180, +180]
    float raw_error = setpoint_deg - current_deg;
    float error = fmod(raw_error + 180.0f, 360.0f) - 180.0f;

    // Provisional derivative (unfiltered)
    float derivative = 0.0f;
    if (dt > 0.0f) {
      float raw_derivative = (error - prev_error) / dt;
      // Simple low‐pass filter on D term
      derivative = der_filter_coeff * raw_derivative
                 + (1.0f - der_filter_coeff) * prev_derivative;
      prev_derivative = derivative;
    }

    // Provisional integral
    float proposed_integral = integral;
    if (dt > 0.0f) {
      proposed_integral += error * dt;
    }

    // Unsaturated output
    float unsat = Kp * error + Ki * proposed_integral + Kd * derivative;

    // Saturate
    float sat = unsat;
    if (sat > output_limit) sat = output_limit;
    if (sat < -output_limit) sat = -output_limit;

    // Anti‐windup: only commit integral if NOT saturated
    if (fabs(unsat) <= output_limit) {
      integral = proposed_integral;
    }

    prev_error = error;
    return sat;
  }

  // Allow external code to adjust gains
  void setGains(float Kp_, float Ki_, float Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
  }
  void getGains(float &outKp, float &outKi, float &outKd) {
    outKp = Kp; outKi = Ki; outKd = Kd;
  }

private:
  float Kp, Ki, Kd;
  float output_limit;
  float integral;
  float prev_error;
  float prev_derivative = 0.0f;
  const float der_filter_coeff = 0.3f;  // α for D‐term filter
  unsigned long prev_time;              // last millis() timestamp
};


// ===== Simple Auto‐Tuner / Stability Monitor =====
class SimpleAutoTuner {
public:
  // window_size = how many consecutive error samples to check
  // error_threshold = if abs(error) > this after window, trigger
  // adjust_factor = multiply Kp/Kd by this factor when instability detected
  SimpleAutoTuner(PID &pid_ref,
                  uint8_t window_size_ = 5,
                  float error_threshold_ = 2.0f,
                  float adjust_factor_ = 0.8f)
    : pid(pid_ref),
      window_size(window_size_),
      error_threshold(error_threshold_),
      adjust_factor(adjust_factor_),
      head(0),
      count(0)
  {
    // initialize error buffer to zeros
    for (uint8_t i = 0; i < MAX_WINDOW; i++) {
      errors[i] = 0.0f;
    }
  }

  // Call this every control step with absolute error (deg):
  void update(float abs_error) {
    // Push new error into circular buffer
    errors[head] = abs_error;
    head = (head + 1) % window_size;
    if (count < window_size) count++;

    // Only proceed if buffer is full
    if (count < window_size) return;

    // Check if errors are non‐decreasing over the entire window
    bool nonDec = true;
    for (uint8_t i = 1; i < window_size; i++) {
      uint8_t idxPrev = (head + MAX_WINDOW - i - 1) % window_size;
      uint8_t idxCurr = (head + MAX_WINDOW - i) % window_size;
      if (errors[idxCurr] < errors[idxPrev]) {
        nonDec = false;
        break;
      }
    }

    // If error never decreased AND latest error exceeds threshold, reduce gains
    float latestError = errors[(head + MAX_WINDOW - 1) % window_size];
    if (nonDec && latestError > error_threshold) {
      float Kp, Ki, Kd;
      pid.getGains(Kp, Ki, Kd);
      float newKp = Kp * adjust_factor;
      float newKi = Ki;  // leave integral alone
      float newKd = Kd * adjust_factor;
      pid.setGains(newKp, newKi, newKd);
      Serial.printf(
        "[AutoTuner] Instability → Kp: %.3f→%.3f, Ki: %.3f→%.3f, Kd: %.3f→%.3f\n",
        Kp, newKp, Ki, newKi, Kd, newKd
      );
      // Clear buffer so we don’t immediately trigger again
      clearBuffer();
    }
  }

private:
  PID &pid;
  static const uint8_t MAX_WINDOW = 10;  // absolute max size
  uint8_t window_size;
  float error_threshold;
  float adjust_factor;

  float errors[MAX_WINDOW];
  uint8_t head;
  uint8_t count;

  void clearBuffer() {
    for (uint8_t i = 0; i < window_size; i++) {
      errors[i] = 0.0f;
    }
    head = 0;
    count = 0;
  }
};



// ================= GET HEADING (haha head) ======================
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
// Example: read current roll angle (deg) from IMU or other sensor:
float readRollSensor() {
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
  //float angle_to_north = fmod((360.0 - yaw_estimate), 360.0);

  return yaw_estimate;
}



// ===== Global Objects & Configuration =====
const float FIN_LIMIT = 15.0f;   // max fin deflection (±15°)
PID pidController(0.6267, 0, 0.9488, FIN_LIMIT);


void applyFinOutput(float finDeg) {
  // Constrain the requested deflection to ±FIN_LIMIT
  if (finDeg >  FIN_LIMIT) finDeg =  FIN_LIMIT;
  if (finDeg < -FIN_LIMIT) finDeg = -FIN_LIMIT;

  // Compute each servo’s absolute position (0°..180°)
  float angleLeft  = 90.0f + finDeg;  // center at 90°, plus finDeg
  float angleRight = 90.0f - finDeg;  // center at 90°, minus finDeg

  // Constrain to valid servo range [0°, 180°]
  angleLeft  = constrain(angleLeft,  0.0f, 180.0f);
  angleRight = constrain(angleRight, 0.0f, 180.0f);

  // Send commands to the servos
  servoLeft.write(static_cast<int>(angleLeft));
  servoRight.write(static_cast<int>(angleRight));
}
// AutoTuner parameters:
const uint8_t TUNER_WINDOW = 5;     // check last 5 error samples
const float   TUNER_THRESH = 5.0f;  // degrees
const float   TUNER_FACTOR = 0.8f;  // reduce Kp/Kd to 80%
SimpleAutoTuner tuner(pidController, TUNER_WINDOW, TUNER_THRESH, TUNER_FACTOR);


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
  // 1) Read current roll (deg)
  float roll = readRollSensor();

  // 2) Compute PID output
  float setpoint = 0.0f;  // we want to hold 0° roll
  float finCmd = pidController.compute(setpoint, roll);

  // 3) Send command to actuator
  applyFinOutput(finCmd);

  // 4) Update tuner with |error|
  float absError = fabs(setpoint - roll);
  tuner.update(absError);

  // 5) Wait until next control step
  //    On ESP32 @115K baud, loop runs fast; use a small delay for ~100 Hz update:
  delay(10);
}
