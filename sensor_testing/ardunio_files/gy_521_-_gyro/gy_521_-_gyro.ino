#include "Wire.h"
//***************************
// whats the difference betweeen angle stability and rotation

//In the provided example code, the usage of A4 and A5 pins is implicit in the Wire.begin() function. 
//The Wire library automatically uses the A4 pin for SDA (data line) and the A5 pin for SCL (clock line)
//on most Arduino boards, as these are the default I2C pins.

//****************************
const int MPU_ADDR = 0x68;

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

char tmp_str[7];

char* convert_int16_to_str(int16_t i) {
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7*2, true);
  
  accelerometer_x = Wire.read()<<8 | Wire.read();
  accelerometer_y = Wire.read()<<8 | Wire.read();
  accelerometer_z = Wire.read()<<8 | Wire.read();
  temperature = Wire.read()<<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();

  // Calculate angle stability and rotation
  float angleX = gyro_x / 131.0;  // 131.0 is the sensitivity scale factor for the gyro (degrees per second)
  float angleY = gyro_y / 131.0;
  float angleZ = gyro_z / 131.0;

  // Print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  //Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  //Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  //Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.print(" | AngleX = "); Serial.print(angleX);
  Serial.print(" | AngleY = "); Serial.print(angleY);
  Serial.print(" | AngleZ = "); Serial.print(angleZ);
  Serial.println();

  delay(1000);
}
