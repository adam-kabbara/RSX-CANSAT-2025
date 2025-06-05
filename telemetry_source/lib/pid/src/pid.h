// PID.h
#ifndef PID_H
#define PID_H
#include <Adafruit_BNO08x.h>
#include <Adafruit_LIS3MDL.h>
class PID {
public:
  PID(float Kp_, float Ki_, float Kd_, float output_limit_);

  float update(float error, float current_time);  // declare update() method
  float compute(float setpoint_deg, float current_deg);

  // Allow external code to adjust gains
  void setGains(float Kp_, float Ki_, float Kd_);

  void getGains(float &outKp, float &outKi, float &outKd);

private:
  float Kp, Ki, Kd;
  float output_limit;
  float integral;
  float prev_error;
  float prev_derivative = 0.0f;
  const float der_filter_coeff = 0.3f;  // α for D‐term filter
  unsigned long prev_time;              // last millis() timestamp
};

class SimpleAutoTuner
{
private:
  PID &pid;
  static const uint8_t MAX_WINDOW = 10;  // absolute max size
  uint8_t window_size;
  float error_threshold;
  float adjust_factor;
  float errors[MAX_WINDOW];
  uint8_t head;
  uint8_t count;

public:
  // window_size = how many consecutive error samples to check
  // error_threshold = if abs(error) > this after window, trigger
  // adjust_factor = multiply Kp/Kd by this factor when instability detected
  SimpleAutoTuner(PID &pid_ref,
                  uint8_t window_size_ = 5,
                  float error_threshold_ = 2.0f,
                  float adjust_factor_ = 0.8f);

  void clearBuffer();

  // Call this every control step with absolute error (deg):
  void update(float abs_error);
};

class PIDController
{
private:
    const float FIN_LIMIT = 15.0f;
    PID *pidController;
    SimpleAutoTuner *tuner;
    const uint8_t TUNER_WINDOW = 5;
    const float   TUNER_THRESH = 5.0f;
    const float   TUNER_FACTOR = 0.8f;
    float yaw_estimate = 0.0;
    float yaw_cov = 1.0;
    const float Q = 0.01;  // process noise
    const float R = 1.0;   // measurement noise
    unsigned long lastUpdate = 0;
  
    // How close to “true north” we call “close enough” (in degrees)
    static constexpr float NORTH_TOLERANCE_DEG = 5.0f;

    // Timeout threshold in milliseconds (e.g. if T = 2 seconds, T_ms = 2000)
    const unsigned long NORTH_TIMEOUT_MS;

    // Timestamp of the last moment yaw_estimate was within ±NORTH_TOLERANCE_DEG of north.
    unsigned long last_time_at_north;
    bool swapped = false; //IF THIS IS FALSE, IT IS TRYING TO POINT NORTH, ELSE IT IS TRYING TO STOP SPINNING

public:
    PIDController(unsigned long north_timeout_ms = 5000UL) //THIS TIMEOUT VARIABLE SAYS HOW LONG IT TAKES UNTIL THE OTHER TUNER KICKS IN
        : NORTH_TIMEOUT_MS(north_timeout_ms),
        last_time_at_north(millis())
    {
        pidController = new PID(0.6267, 0, 0.9488, FIN_LIMIT);
        tuner = new SimpleAutoTuner(*pidController, TUNER_WINDOW, TUNER_THRESH, TUNER_FACTOR);
        lastUpdate = millis();
    }
    void kalmanUpdate(float gyro_z, float mag_yaw, float dt);
    void update_PID(float ax, float ay, float az, float gyroZ, float mx, float my, float mz, float* fin_left, float* fin_right);
    bool hasMissedNorthTooLong(float yaw_estimate);
    float quaternionToYawDegrees(float r, float i, float j, float k);
    float computeTiltCompensatedYaw(float mx, float my, float mz,float ax, float ay, float az);
};

#endif
