#include "pid.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_LIS3MDL.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

PID::PID(float Kp_, float Ki_, float Kd_, float output_limit_)
    : Kp(Kp_), Ki(Ki_), Kd(Kd_), output_limit(output_limit_),
    integral(0.0f), prev_error(0.0f), prev_time(0.0f)
{}

float PID::compute(float setpoint_deg, float current_deg)
{
    unsigned long now = millis();
    float dt = (prev_time == 0) ? 0.0f : float(now - prev_time) / 1000.0f;
    prev_time = now;

    // Compute angular error in [–180, +180]
    float raw_error = setpoint_deg - current_deg;
    float error = fmod(raw_error + 180.0f, 360.0f) - 180.0f;

    // Provisional derivative (unfiltered)
    float derivative = 0.0f;
    if(dt > 0.0f) {
        float raw_derivative = (error - prev_error) / dt;
        // Simple low‐pass filter on D term
        derivative = der_filter_coeff * raw_derivative
                    + (1.0f - der_filter_coeff) * prev_derivative;
        prev_derivative = derivative;
    }

    // Provisional integral
    float proposed_integral = integral;
    if(dt > 0.0f) {
        proposed_integral += error * dt;
    }

    // Unsaturated output
    float unsat = Kp * error + Ki * proposed_integral + Kd * derivative;

    // Saturate
    float sat = unsat;
    if(sat > output_limit) sat = output_limit;
    if(sat < -output_limit) sat = -output_limit;

    // Anti‐windup: only commit integral if NOT saturated
    if(fabs(unsat) <= output_limit) {
        integral = proposed_integral;
    }

    prev_error = error;
    return sat;
}

// Allow external code to adjust gains
void PID::setGains(float Kp_, float Ki_, float Kd_)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}

void PID::getGains(float &outKp, float &outKi, float &outKd)
{
    outKp = Kp; outKi = Ki; outKd = Kd;
}

SimpleAutoTuner::SimpleAutoTuner(PID &pid_ref,
                uint8_t window_size_,
                float error_threshold_,
                float adjust_factor_)
    : pid(pid_ref),
    window_size(window_size_),
    error_threshold(error_threshold_),
    adjust_factor(adjust_factor_),
    head(0),
    count(0)
{
    // initialize error buffer to zeros
    for(uint8_t i = 0; i < MAX_WINDOW; i++)
    {
        errors[i] = 0.0f;
    }
}

void SimpleAutoTuner::clearBuffer()
{
    for(uint8_t i = 0; i < window_size; i++)
    {
        errors[i] = 0.0f;
    }
    head = 0;
    count = 0;
}

// Call this every control step with absolute error (deg):
void SimpleAutoTuner::update(float abs_error) 
{
    // Push new error into circular buffer
    errors[head] = abs_error;
    head = (head + 1) % window_size;
    if(count < window_size) count++;

    // Only proceed if buffer is full
    if(count < window_size) return;

    // Check if errors are non‐decreasing over the entire window
    bool nonDec = true;
    for(uint8_t i = 1; i < window_size; i++) 
    {
        uint8_t idxPrev = (head + MAX_WINDOW - i - 1) % window_size;
        uint8_t idxCurr = (head + MAX_WINDOW - i) % window_size;
        if(errors[idxCurr] < errors[idxPrev])
        {
            nonDec = false;
            break;
        }
    }

    // If error never decreased AND latest error exceeds threshold, reduce gains
    float latestError = errors[(head + MAX_WINDOW - 1) % window_size];
    if(nonDec && latestError > error_threshold) 
    {
        float Kp, Ki, Kd;
        pid.getGains(Kp, Ki, Kd);
        float newKp = Kp * adjust_factor;
        float newKi = Ki;  // leave integral alone
        float newKd = Kd * adjust_factor;
        pid.setGains(newKp, newKi, newKd);
        // Clear buffer so we don’t immediately trigger again
        clearBuffer();
    }
}

// Convert BNO08x quaternion to yaw (degrees)
float PIDController::quaternionToYawDegrees(float r, float i, float j, float k)
{
    float yaw = atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k));
    float yaw_deg = yaw * 180.0 / PI;
    if(yaw_deg < 0) yaw_deg += 360.0;
    return yaw_deg;
}

// Tilt-compensated heading from magnetometer + accelerometer
float PIDController::computeTiltCompensatedYaw(float mx, float my, float mz,
                                float ax, float ay, float az)
{
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
    if(heading_deg < 0) heading_deg += 360.0;
    return heading_deg;
}

// Kalman filter update
void PIDController::kalmanUpdate(float gyro_z, float mag_yaw, float dt)
{
    yaw_estimate += gyro_z * dt * 180.0 / PI;  // convert rad/s to deg/s
    yaw_cov += Q;

    yaw_estimate = fmod(yaw_estimate, 360.0f);
    if(yaw_estimate < 0) yaw_estimate += 360.0f;

    float K = yaw_cov / (yaw_cov + R);
    yaw_estimate += K * (mag_yaw - yaw_estimate);
    yaw_cov *= (1 - K);
}

bool PIDController::hasMissedNorthTooLong(float yaw_estimate)
{
    unsigned long now = millis();

    // Normalize yaw_estimate into [0, 360)
    float y = fmodf(yaw_estimate, 360.0f);
    if(y < 0.0f) y += 360.0f;

    // How far is y from 0°?  (take the shorter wraparound distance)
    float delta_to_north = min(y, 360.0f - y);

    if(delta_to_north <= NORTH_TOLERANCE_DEG) {
      // We’re “close enough” to north—reset the timer
      last_time_at_north = now;
    }

    // If it’s been more than NORTH_TIMEOUT_MS since we were “close enough”:
    return (now - last_time_at_north) > NORTH_TIMEOUT_MS;
  }


void PIDController::update_PID(float ax, float ay, float az, float gyroZ, float mx, float my, float mz, float* fin_left, float* fin_right)
{
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;
    
    float magYaw = computeTiltCompensatedYaw(mx, my, mz, ax, ay, az);

    // Kalman filter
    kalmanUpdate(gyroZ, magYaw, dt);
    float finCmd, error;
    float setpoint = 0.0f;
    if(!swapped)
    {
      finCmd = pidController->compute(setpoint, yaw_estimate);
      error = fmod((setpoint - yaw_estimate + 540.0f), 360.0f) - 180.0f;
    }
    else
    {
      finCmd = pidController->compute(setpoint, gyroZ* 180.0 / PI);
      error = fmod((setpoint - gyroZ* 180.0 / PI + 540.0f), 360.0f) - 180.0f;
    }
    tuner->update(fabs(error)); //THIS IS THE AUTOTUNER WHAT IT DOES IS IT WEAKENS THE GAINS OF THE PID IF IT THINKS ITS GOING UNSTABLE
    if((!swapped) && hasMissedNorthTooLong(yaw_estimate))
    {
        pidController->setGains(0.6267f, 0, 0.9488f);        
        swapped = true;
        tuner->clearBuffer();
    }
    
    if (finCmd >  FIN_LIMIT) finCmd =  FIN_LIMIT;
    if (finCmd < -FIN_LIMIT) finCmd = -FIN_LIMIT;

    // Apply the change incrementally to the current angles
    *fin_left  += finCmd;
    //*fin_right -= finCmd;

    // Constrain to valid servo range [0°, 180°]
    *fin_left  = constrain(*fin_left,  0.0f, 180.0f);
    *fin_right = 180.0f - (*fin_left);
}

float PIDController::getYawEstimate() const
{
    return yaw_estimate;
}
