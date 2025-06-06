#include <Arduino.h>
#include <Servo.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
#define PI 3.1415926535897932384626433832795

// #define FAST_MODE

// For SPI mode, we also need a RESET
#define BNO08X_RESET 5
// but not for I2C or UART
//#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

Servo myservo;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup(void) {
  myservo.attach(3);
  Serial.begin(115200);
  //hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent)

  /*
  delay(200);
  digitalWrite(BNO08X_RESET, LOW);
  delay(200);
  digitalWrite(BNO08X_RESET, HIGH);
  delay(20);
  */


  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  //if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");



  Serial.println("Reading events");
  delay(100);
  bno08x.enableReport(reportType, reportIntervalUs);


  //magnometer setup
  Serial.println("Setting magn f calib");
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  Serial.println("setting raw magn");
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
  Serial.println("donemagninit");
  //setReports(reportType, reportIntervalUs);
  pinMode(BNO08X_RESET, OUTPUT);
  digitalWrite(BNO08X_RESET, LOW);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}



int pos = 0;

double err;
double error_sum = 0;
double prev_error = 0;

double ki = 1.0, kd = 1.0, kp = 1.0;

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now = micros();
    //Serial.print(now - last);             Serial.print("\t");
    last = now;
    /*Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.print(ypr.roll);             Serial.print("\t");*/

    // magnetic feild calc
    double x = sensorValue.un.magneticField.x;
    double y = sensorValue.un.magneticField.y;

    //atan2 ret                                                                                                       urns the angle in radians, then its converted to degrees -180 to 180, then mapped from 0 to 360
    //double magCircleAngle = map(atan2(x-0.03*(1.62/1.99),y+0.005*(1.99/1.62))*180.0/PI, -180, 180, 0, 360);
    double magCircleAngle = atan2(x - 0.03 * (1.62 / 1.99), y + 0.005 * (1.99 / 1.62)) * 180.0 / PI;
    err = magCircleAngle;

    error_sum += err;
    pos -= err * kp + error_sum * ki + ((err - prev_error) / (1)) * kd;
    prev_error = err;
    //myservo.write(pos);
    Serial.print("magdeg_map: ");
    Serial.println(magCircleAngle);
    Serial.print("motor angle: ");
    Serial.println(pos);
  }
}
