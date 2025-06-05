#include "sensorManager.h"

SensorManager::SensorManager()
{
    GPS_Serial = new HardwareSerial(1);
    mySPI = new SPIClass(VSPI);
    bno08x = new Adafruit_BNO08x(BNO_RST_PIN);
    rtc = new uRTCLib(0x68);
}

OperatingState SensorManager::updateState(OperatingState curr_state, MissionManager &mission_info)
{
    int idx = alt_data.idx - 1;
    int size = alt_data.window_size;

    switch(curr_state)
    {
        case LAUNCH_PAD: {

            // Average of last 5 samples is > 5m

            float alt_sum = 0.0;

            for (int i = 0; i < 5; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 5;

            if(alt_sum > 5.0)
            {
                mission_info.setOpState(ASCENT);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(ASCENT);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                return ASCENT;
            }
            break;
        }
        
        case ASCENT: {
            // Because sim mode only sends reading every 1 second it's way harder to detect apogee
            if(mission_info.getOpMode() == OPMODE_SIM)
            {
                // Just check current reading is <10m difference from last reading
                if(alt_data.buffer[idx] - alt_data.buffer[(idx - 1 + size) % size] < 5)
                {
                    alt_data.max_alt = alt_data.buffer[idx];
                    mission_info.setOpState(APOGEE);
                    mission_info.beginPref("xb-set", false);
                    int state_int = static_cast<int>(APOGEE);
                    mission_info.putPrefInt("opstate", state_int);
                    mission_info.endPref();
                    return APOGEE;
                }
            }
            else
            {
                if(alt_data.sample_count > size)
                {
                    // To detect apogee, take last three averages of three samples at 0.15sec intervals
                    // and determine if the difference between each of them is less than 5m

                    int step = 0.15 * SENSOR_SAMPLE_RATE_HZ;

                    float avg0 = (alt_data.buffer[(idx + size - 1) % size] +
                                alt_data.buffer[idx] +
                                alt_data.buffer[(idx + 1) % size]) / 3.0;

                    float avg1 = (alt_data.buffer[(idx - step + size - 1) % size] +
                                alt_data.buffer[(idx - step + size) % size] +
                                alt_data.buffer[(idx - step + size + 1) % size]) / 3.0;

                    float avg2 = (alt_data.buffer[(idx - 2 * step + size - 1) % size] +
                                alt_data.buffer[(idx - 2 * step + size) % size] +
                                alt_data.buffer[(idx - 2 * step + size + 1) % size]) / 3.0;

                    float d1 = fabs(avg0 - avg1);
                    float d2 = fabs(avg1 - avg2);

                    if (d1 < 5.0 && d2 < 5.0)
                    {
                        alt_data.max_alt = avg1;
                        mission_info.setOpState(APOGEE);
                        mission_info.beginPref("xb-set", false);
                        int state_int = static_cast<int>(APOGEE);
                        mission_info.putPrefInt("opstate", state_int);
                        mission_info.endPref();
                        return APOGEE;
                    }

                    // Check we are descending in case apogee was missed

                    step = 0.25 * SENSOR_SAMPLE_RATE_HZ;

                    avg0 = (alt_data.buffer[(idx + size - 1) % size] +
                                alt_data.buffer[idx] +
                                alt_data.buffer[(idx + 1) % size]) / 3.0;

                    avg1 = (alt_data.buffer[(idx - step + size - 1) % size] +
                                alt_data.buffer[(idx - step + size) % size] +
                                alt_data.buffer[(idx - step + size + 1) % size]) / 3.0;

                    avg2 = (alt_data.buffer[(idx - 2 * step + size - 1) % size] +
                                alt_data.buffer[(idx - 2 * step + size) % size] +
                                alt_data.buffer[(idx - 2 * step + size + 1) % size]) / 3.0;

                    // Replace max alt if needed
                    if(avg0 > alt_data.max_alt)
                    {
                        alt_data.max_alt = avg0;
                    }

                    if (avg0 < avg1 && avg1 < avg2)
                    {
                        mission_info.setOpState(DESCENT);
                        mission_info.beginPref("xb-set", false);
                        int state_int = static_cast<int>(DESCENT);
                        mission_info.putPrefInt("opstate", state_int);
                        mission_info.endPref();
                        return DESCENT;
                    }
                }
            }
            break;
        }

        case APOGEE: {
            if(mission_info.getOpMode() == OPMODE_SIM)
            {
                // Just check last 3 readings are decreasing
                float r0 = alt_data.buffer[(idx + size - 2) % size];
                float r1 = alt_data.buffer[(idx + size - 1) % size];
                float r2 = alt_data.buffer[idx];

                if (r0 > r1 && r1 > r2)
                {
                    alt_data.max_alt = alt_data.buffer[idx];
                    mission_info.setOpState(DESCENT);
                    mission_info.beginPref("xb-set", false);
                    int state_int = static_cast<int>(DESCENT);
                    mission_info.putPrefInt("opstate", state_int);
                    mission_info.endPref();
                    return DESCENT;
                }
            }
            else
            {
                // Last three average of 3 sample reading over 0.25 sec intervals are decreasing

                int step = 0.25 * SENSOR_SAMPLE_RATE_HZ;

                float avg0 = (alt_data.buffer[(idx + size - 1) % size] +
                            alt_data.buffer[idx] +
                            alt_data.buffer[(idx + 1) % size]) / 3.0;

                float avg1 = (alt_data.buffer[(idx - step + size - 1) % size] +
                            alt_data.buffer[(idx - step + size) % size] +
                            alt_data.buffer[(idx - step + size + 1) % size]) / 3.0;

                float avg2 = (alt_data.buffer[(idx - 2 * step + size - 1) % size] +
                            alt_data.buffer[(idx - 2 * step + size) % size] +
                            alt_data.buffer[(idx - 2 * step + size + 1) % size]) / 3.0;

                // Replace max alt if needed
                if(avg0 > alt_data.max_alt)
                {
                    alt_data.max_alt = avg0;
                }

                if (avg0 < avg1 && avg1 < avg2)
                {
                    mission_info.setOpState(DESCENT);
                    mission_info.beginPref("xb-set", false);
                    int state_int = static_cast<int>(DESCENT);
                    mission_info.putPrefInt("opstate", state_int);
                    mission_info.endPref();
                    return DESCENT;
                }
                break;
            }
        }

        case DESCENT: {
            // Average of last 5 readings are 75% of max

            float alt_sum = 0.0;

            for (int i = 0; i < 5; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 5;

            if (alt_sum <= 0.75 * alt_data.max_alt)
            {
                writeReleaseServo(38);
                mission_info.setOpState(PROBE_RELEASE);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(PROBE_RELEASE);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                return PROBE_RELEASE;
            }
            break;
        }

        case PROBE_RELEASE: {

            // Average of last 5 samples is < 5m

            float alt_sum = 0.0;

            for (int i = 0; i < 5; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 5;

            if(alt_sum < 5.0)
            {
                mission_info.setOpState(LAUNCH_PAD);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(LAUNCH_PAD);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                return LAUNCH_PAD;
            }
            break;
        }

    }

    return curr_state;
}

void SensorManager::sampleSensors(MissionManager &mission_info, SerialManager &ser)
{
    // PRESSURE
    if(mission_info.getOpMode() == OPMODE_SIM)
    {
      send_packet.PRESSURE = mission_info.getSimpData()/1000.0;
    }
    else
    {
      send_packet.PRESSURE = getPressure();
    }

    // ALTITUDE
    send_packet.ALTITUDE = pressure_to_alt(send_packet.PRESSURE*10.0) - mission_info.getLaunchAlt();

    // STATE
    if(mission_info.getOpMode() == OPMODE_FLIGHT)
    {
        alt_data.buffer[alt_data.idx] = send_packet.ALTITUDE;
        alt_data.idx = (alt_data.idx + 1) % alt_data.window_size;
        alt_data.sample_count++;
    }

    strcpy(send_packet.STATE, op_state_to_string(updateState(mission_info.getOpState(), mission_info)));

    // TEAM ID
    send_packet.TEAM_ID_PCKT = TEAM_ID;

    // MODE
    strcpy(send_packet.MODE, op_mode_to_string(mission_info.getOpMode(), 0));

    int ran_num[17];
    for(int i = 0; i < 17; i++)
    {
      ran_num[i] = random(0, 20);
    }

    getRtcTime(send_packet.MISSION_TIME);
    send_packet.TEMPERATURE = getTemp();
    
    send_packet.VOLTAGE = getVoltage();

    // IMU
    for(int i = 0; i < 3; i++)
    {
        sh2_SensorValue_t sensorValue;
        if(bno08x->getSensorEvent(&sensorValue))
        {
            switch(sensorValue.sensorId)
            {
                case SH2_ACCELEROMETER:
                    send_packet.ACCEL_R = sensorValue.un.accelerometer.x;
                    send_packet.ACCEL_P = sensorValue.un.accelerometer.y;
                    send_packet.ACCEL_Y = sensorValue.un.accelerometer.z;
                    ax = sensorValue.un.accelerometer.x;
                    ay = sensorValue.un.accelerometer.y;
                    az = sensorValue.un.accelerometer.z;
                    break;
                
                case SH2_GYROSCOPE_CALIBRATED:
                    send_packet.GYRO_R = sensorValue.un.gyroscope.y * RAD_TO_DEG;
                    send_packet.GYRO_P = sensorValue.un.gyroscope.x * RAD_TO_DEG;
                    send_packet.GYRO_Y = sensorValue.un.gyroscope.z * RAD_TO_DEG;
                    gyroZ = sensorValue.un.gyroscope.z;
                    break;
                
                case SH2_ROTATION_VECTOR:
                    float qw = sensorValue.un.rotationVector.real;
                    float qx = sensorValue.un.rotationVector.i;
                    float qy = sensorValue.un.rotationVector.j;
                    float qz = sensorValue.un.rotationVector.k;

                    // Convert quaternion to Euler angles (roll, pitch, yaw in radians)
                    float roll  = atan2(2.0f * (qw * qx + qy * qz),
                                        1.0f - 2.0f * (qx * qx + qy * qy));
                    float pitch = asin(2.0f * (qw * qy - qz * qx));
                    float yaw   = atan2(2.0f * (qw * qz + qx * qy),
                                        1.0f - 2.0f * (qy * qy + qz * qz));

                    // Convert radians to degrees
                    roll  *= 180.0 / PI;
                    pitch *= 180.0 / PI;
                    yaw   *= 180.0 / PI;

                    sensors_event_t event_lis3mdl;
                    lis3mdl.getEvent(&event_lis3mdl);
                    mx = event_lis3mdl.magnetic.x;
                    my = event_lis3mdl.magnetic.y;
                    mz = event_lis3mdl.magnetic.z;

                    float mx_g = mx / 100;
                    float my_g = my / 100;
                    float mz_g = mz / 100;

                    float cr = cos(roll),  sr = sin(roll);
                    float cp = cos(pitch), sp = sin(pitch);
                    float cy = cos(yaw),   sy = sin(yaw);

                    // Rotation matrix from sensor (XYZ) to RPY frame
                    float R[3][3] = {
                    { cp * cy,                         cp * sy,                        -sp      },
                    { sr * sp * cy - cr * sy,         sr * sp * sy + cr * cy,         sr * cp },
                    { cr * sp * cy + sr * sy,         cr * sp * sy - sr * cy,         cr * cp }
                    };

                    // Rotate magnetic vector
                    float mag_r = R[0][0]*mx_g + R[0][1]*my_g + R[0][2]*mz_g;
                    float mag_p = R[1][0]*mx_g + R[1][1]*my_g + R[1][2]*mz_g;
                    float mag_y = R[2][0]*mx_g + R[2][1]*my_g + R[2][2]*mz_g;

                    send_packet.MAG_R = mag_r;
                    send_packet.MAG_P = mag_p;
                    send_packet.MAG_Y = mag_y;

                    break;
                    
            }
        }
    }

    /*
    float servo_left = 0.0;
    float servo_right = 0.0;
    pid_cntl.update_PID(ax, ay, az, gyroZ, mx, my, mz, &servo_left, &servo_right);
    unsigned long servo_now = millis();
    if(servo_now - last_servo_update >= 250)
    {
        //ser.sendInfoDataMsg("GyroZ=%f",gyroZ);
        //ser.sendInfoDataMsg("servo_left=%f",servo_left);
        //ser.sendInfoDataMsg("servo_right=%f",servo_right);
        last_servo_update = servo_now;
    }
    */

    send_packet.AUTO_GYRO_ROTATION_RATE = getRotRate();

    // int gps_avail = GPS_Serial->available();
    // for(int i = 0; i < gps_avail; i++)
    // {
    //     gps.encode(GPS_Serial->read());
    // }

    int gps_avail = GPS_Serial->available();
    gpsData = "";
    for (int i = 0; i < gps_avail; i++) {
        gpsData += (char)GPS_Serial->read();
    }

    if (gpsData.length() > 0) {
        for (size_t i = 0; i < gpsData.length(); i++) {
            gps.encode(gpsData[i]);
        }
    }

    if (gps.location.isUpdated())
    {
        getGpsLat(&send_packet.GPS_LATITUDE);
        getGpsLong(&send_packet.GPS_LONGITUDE);
    }

    if (gps.time.isUpdated())
    {
        getGpsTime(send_packet.GPS_TIME);
    }

    if (gps.altitude.isUpdated())
    {
        getGpsAlt(&send_packet.GPS_ALTITUDE);
    }

    if (gps.satellites.isUpdated())
    {
        getGpsSats(&send_packet.GPS_SATS);
    }

    int cam1_state = digitalRead(CAMERA1_STATUS_PIN);
    int cam2_state = digitalRead(CAMERA2_STATUS_PIN);

    if (cam1_state && cam2_state)
    {
        send_packet.CAMERA_STATUS = 3;
    }
    else if (cam1_state)
    {
        send_packet.CAMERA_STATUS = 1;
    }
    else if (cam2_state)
    {
        send_packet.CAMERA_STATUS = 2;
    }
    else
    {
        send_packet.CAMERA_STATUS = 0;
    }
}

void SensorManager::build_data_str(char *buff, size_t size)
{
    snprintf(buff, size,
        "%d,%s,%d,%s,%s,"
        "%.1f,%.1f,%.1f,%.1f,%d,"
        "%d,%d,%d,%d,%d,"
        "%.1f,%.1f,%.1f,%.1f,%s,"
        "%.1f,%.4f,%.4f,%d,%s,"
        "%d",
        send_packet.TEAM_ID_PCKT, 
        send_packet.MISSION_TIME, 
        send_packet.PACKET_COUNT, 
        send_packet.MODE, 
        send_packet.STATE,
        send_packet.ALTITUDE, 
        send_packet.TEMPERATURE, 
        send_packet.PRESSURE, 
        send_packet.VOLTAGE, 
        send_packet.GYRO_R,
        send_packet.GYRO_P, 
        send_packet.GYRO_Y, 
        send_packet.ACCEL_R,
        send_packet.ACCEL_P, 
        send_packet.ACCEL_Y, 
        send_packet.MAG_R, 
        send_packet.MAG_P, 
        send_packet.MAG_Y, 
        send_packet.AUTO_GYRO_ROTATION_RATE, 
        send_packet.GPS_TIME,
        send_packet.GPS_ALTITUDE, 
        send_packet.GPS_LATITUDE, 
        send_packet.GPS_LONGITUDE, 
        send_packet.GPS_SATS, 
        send_packet.CMD_ECHO,
        send_packet.CAMERA_STATUS); 
}

const char* SensorManager::op_state_to_string(OperatingState state) 
{
    static const char* states[] = {
        "LAUNCH_PAD",
        "ASCENT",
        "APOGEE",
        "DESCENT",
        "PROBE_RELEASE",
        "LANDED",
        "IDLE"
    };

    return states[state];

}

const char* SensorManager::op_mode_to_string(OperatingMode mode, int full) 
{
    if (full == 1)
    {
        if(mode == OPMODE_FLIGHT)
        {
            return "FLIGHT";
        }
        else
        {
            return "SIM";
        }
    }
    else
    {
        if(mode == OPMODE_FLIGHT)
        {
            return "F";
        }
        else
        {
            return "S";
        }
    }
}

float SensorManager::pressure_to_alt(const float pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}


void SensorManager::cmd_buff_to_echo(char *cmd_buff) 
{
    int comma = 0;
    int echo_indx = 0;

    for (int i = 0; cmd_buff[i] != '\0'; i++)
    {
        if (cmd_buff[i] == ',')
        {
            comma++;
            continue;
        }

        if (comma > 1)
        {
            send_packet.CMD_ECHO[echo_indx++] = cmd_buff[i];
        }
    }

    send_packet.CMD_ECHO[echo_indx] = '\0';
}

void SensorManager::setAltData(float alt)
{
    alt_data.buffer[alt_data.idx] = alt;
    alt_data.idx = (alt_data.idx + 1) % alt_data.window_size;
    alt_data.sample_count++;
}

void SensorManager::resetAltData()
{
    for(int i = 0; i < alt_data.window_size; i++)
    {
        alt_data.buffer[i] = 0.0;
    }
    alt_data.idx = 0;
    alt_data.current_alt = 0.0;
    alt_data.max_alt = 0.0;
    alt_data.sample_count = 0;
}

void SensorManager::startSensors(SerialManager &ser, MissionManager &info)
{
    // Temperature and Pressure
    uint8_t status = bme.begin(0x77);
    ser.sendInfoDataMsg("Initialized BME with status: %0d", status);
    delay(100);

    // Hall Effect Sensor
    pinMode(HALL_SENSOR_PIN, INPUT);
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    delay(100);
    ser.sendInfoMsg("Initialized hall sensor...");
    
    // Release Servo
    m_servo_release.attach(SERVO_RELEASE_PIN);
    delay(1000);
    if(info.getOpState() == IDLE)
    {
        writeReleaseServo(38);
        ser.sendInfoMsg("Waiting 10 seconds for release servo setup...");
        delay(10000);
        writeReleaseServo(0);
    }
    delay(100);
    ser.sendInfoMsg("Initialized release servo...");

    // GPS
    GPS_Serial = new HardwareSerial(1);
    GPS_Serial->begin(9600, SERIAL_8N1, RX1_PIN, TX1_PIN);
    delay(100);
    ser.sendInfoMsg("Initialized GPS...");
    
    // Gyro Servo 2
    m_servo_gyro_1.attach(SERVO_GYRO1_PIN);
    delay(1000);
    if(info.getOpState() == IDLE)
    {
        writeGyroServoLeft(90);
        delay(100);
    }
    
    // Gyro Servo 1
    m_servo_gyro_2.attach(SERVO_GYRO2_PIN);
    delay(1000);
    if(info.getOpState() == IDLE)
    {
        writeGyroServoRight(90);
        delay(100);
    }
    
    // Camera Servo
    m_servo_camera.attach(SERVO_CAMERA_PIN);
    delay(1000);
    if(info.getOpState() == IDLE)
    {
        writeCameraServo(90);
        delay(100);
    }
    ser.sendInfoMsg("Initialized other servos...");
    
    // Camera Signal Pins (Trigger)
    pinMode(CAMERA1_SIGNAL_PIN, OUTPUT);
    delay(100);
    pinMode(CAMERA2_SIGNAL_PIN, OUTPUT);
    delay(100);
    
    // Camera Data Pins (Recording Status)
    pinMode(CAMERA1_STATUS_PIN, INPUT);
    delay(100);
    pinMode(CAMERA2_STATUS_PIN, INPUT);
    delay(100);
    ser.sendInfoMsg("Initialized camera...");

    mySPI->begin(BNO_SPI_SCK, BNO_SPI_MISO, BNO_SPI_MOSI);
    if (!bno08x->begin_SPI(BNO_CS_PIN, BNO_INT_PIN, mySPI)) 
    {
        ser.sendErrorMsg("BNO was not detected.");
    }
    else
    {
        ser.sendInfoMsg("BNO initialized over SPI");
    }
    
    bno08x->enableReport(SH2_ARVR_STABILIZED_RV);
    bno08x->enableReport(SH2_ACCELEROMETER);
    bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED);
    bno08x->enableReport(SH2_ROTATION_VECTOR);

    if (!lis3mdl.begin_I2C()) 
    {
        ser.sendErrorMsg("LIS3MDL was not detected.");
    }
    else
    {
        ser.sendInfoMsg("LIS3MDL initialized over I2C");
    }

    lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setIntThreshold(500);

    Wire.begin(RTC_I2C_SDA, RTC_I2C_SCL);
    Wire.beginTransmission(0x68);
	byte error = Wire.endTransmission();

    if (error == 0) 
    {
		ser.sendInfoMsg("RTC initialized over I2C");
	} 
    else 
    {
		ser.sendInfoDataMsg("RTC was not detected");
	}

    ser.sendInfoMsg("Done.");
}

void SensorManager::setPacketCount(int count)
{
    send_packet.PACKET_COUNT = count;
}

float SensorManager::getPressure()
{
    return (bme.readPressure() / 1000.0);
}

float SensorManager::getTemp()
{
    return bme.readTemperature();
}

void SensorManager::writeReleaseServo(int pos)
{
    m_servo_release.write(pos);
}

void SensorManager::writeGyroServoRight(int pos)
{
    m_servo_gyro_1.write(pos);
}

void SensorManager::writeGyroServoLeft(int pos)
{
    m_servo_gyro_2.write(pos);
}

void SensorManager::writeCameraServo(int pos)
{
    m_servo_camera.write(pos);
}

void SensorManager::getGpsAlt(float *alt)
{
    if (gps.altitude.isValid())
    {
        *alt = gps.altitude.meters();
    }
}

void SensorManager::getGpsLat(float *lat)
{
    if (gps.location.isValid())
    {
        *lat = gps.location.lat();
    }
}

void SensorManager::getGpsLong(float *lon)
{
    if (gps.location.isValid())
    {
        *lon = gps.location.lng();
    }
}

void SensorManager::getGpsSats(int *sat)
{
    if (gps.satellites.isValid())
    {
        *sat = gps.satellites.value();
    }
}

void SensorManager::getRtcTime(char time_str[DATA_SIZE])
{
    rtc->refresh();

    snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", rtc->hour(), rtc->minute(), rtc->second());
}

void SensorManager::getGpsTime(char time_str[DATA_SIZE])
{   
    if (gps.time.isValid())
    {
        snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    }
}

float SensorManager::getVoltage()
{
    int adc_raw = analogRead(ADC_VOLTAGE_PIN);
    float adc_voltage = ((adc_raw / 4095.0) * 3.3) + 0.15;
    float real_voltage = 
        -224.72385 * pow(adc_voltage, 4) +
        1942.37477 * pow(adc_voltage, 3) -
        6281.94345 * pow(adc_voltage, 2) +
        9018.07141 * adc_voltage -
        4845.86174 + 0.1;
    return real_voltage;
}

float SensorManager::getRotRate()
{
    int analogValue = analogRead(HALL_SENSOR_PIN);
    int currState = (analogValue < HALL_SENSOR_THRESHOLD) ? LOW : HIGH;

    if(currState == LOW && lastState == HIGH)
    {
        digitalWrite(ONBOARD_LED_PIN, HIGH);
        unsigned long now = micros();
        pulseInterval = now - lastPulseTime;
        lastPulseTime = now;
        if(pulseInterval > 0)
        {
            currRPM = 60.0 * 1000000.0 / pulseInterval;
            if(currRPM > 2000.0)
            {
                currRPM = prevRPM;
            }
            prevRPM = currRPM;
        }
    } 
    else
    {
        digitalWrite(ONBOARD_LED_PIN, LOW);
    }

    lastState = currState;
    return currRPM * 6.0;
}

void SensorManager::setRtcTime(int sec, int minute, int hour)
{
    rtc->set(sec, minute, hour, 1, 1, 1, 0);
}
