#include "ControlModule.h"
#include <ESP32Servo.h>
#include <math.h>

// Servo pin config
static const int SERVO_LEFT_PIN  = 2;
static const int SERVO_RIGHT_PIN = 4;

// Servo objects
Servo servoLeft;
Servo servoRight;

// PID class (same as you wrote)
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

// PID instance
const float FIN_LIMIT = 15.0f;
PID pidController(0.6267, 0, 0.9488, FIN_LIMIT);

// AutoTuner instance
const uint8_t TUNER_WINDOW = 5;
const float   TUNER_THRESH = 5.0f;
const float   TUNER_FACTOR = 0.8f;
SimpleAutoTuner tuner(pidController, TUNER_WINDOW, TUNER_THRESH, TUNER_FACTOR);

// Fin command function
void applyFinOutput(float finDeg) {
  finDeg = constrain(finDeg, -FIN_LIMIT, FIN_LIMIT);
  float angleLeft  = constrain(90.0f + finDeg,  0.0f, 180.0f);
  float angleRight = constrain(90.0f - finDeg, 0.0f, 180.0f);
  servoLeft.write(static_cast<int>(angleLeft));
  servoRight.write(static_cast<int>(angleRight));
}

// Setup function
void setup_servos() {
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
}

// PID update call
void update_PID(float roll) {
  float setpoint = 0.0f;
  float finCmd = pidController.compute(setpoint, roll);
  applyFinOutput(finCmd);
  tuner.update(fabs(setpoint - roll));
}


// Kalman filter state
float yaw_estimate = 0.0;
float yaw_cov = 1.0;
const float Q = 0.01;  // process noise
const float R = 1.0;   // measurement noise
unsigned long lastUpdate = 0;

// Kalman filter update
float kalmanUpdate(float gyro_z, float mag_yaw, float dt) {
  yaw_estimate += gyro_z * dt * 180.0 / PI;  // integrate gyro
  yaw_cov += Q;                              // predict

  // Normalize yaw to [0, 360)
  if (yaw_estimate < 0) yaw_estimate += 360.0;
  if (yaw_estimate >= 360.0) yaw_estimate -= 360.0;

  // Kalman gain and update
  float K = yaw_cov / (yaw_cov + R);
  float angle_error = fmod((mag_yaw - yaw_estimate + 540.0), 360.0) - 180.0;
  yaw_estimate += K * angle_error;
  yaw_cov *= (1 - K);

  // Normalize again if needed
  if (yaw_estimate < 0) yaw_estimate += 360.0;
  if (yaw_estimate >= 360.0) yaw_estimate -= 360.0;
  return yaw_estimate;
}
