#include "sensorManager.h"

SensorManager::SensorManager()
{
    GPS_Serial = new HardwareSerial(1);
    mySPI = new SPIClass(VSPI);
    bno08x = new Adafruit_BNO08x(BNO_RST_PIN);
    rtc = new uRTCLib(0x68);
}

OperatingState SensorManager::updateState(OperatingState curr_state, MissionManager &mission_info, SerialManager &ser)
{
    int size = alt_data.window_size;
    int idx = (alt_data.idx - 1) % size;

    if(curr_state != DESCENT && (alt_data.buffer[idx] > alt_data.max_alt) && (alt_data.buffer[idx] < 1000))
    {
        alt_data.max_alt = alt_data.buffer[idx];
    }

    switch(curr_state)
    {
        case LAUNCH_PAD: {

            std::vector<float> alt_values;

            for (int i = 0; i < 2; i++)
            {
                int index = (idx - i + size) % size;
                alt_values.push_back(alt_data.buffer[index]);
            }

            std::sort(alt_values.begin(), alt_values.end());

            float median = alt_values[1];

            if(alt_data.sample_count > 3 && median > 10)
            {
                mission_info.setOpState(ASCENT);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(ASCENT);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                ser.sendInfoDataMsg("Changing state to ASCENT");
                return ASCENT;
            }
            break;
        }
        
        case ASCENT: {

            // Check current reading is less than prev reading and also close to last reading  
            float ra = alt_data.buffer[idx];
            float rb = alt_data.buffer[(idx - 1 + size) % size];
            if(ra < (rb-10) && fabs(rb - ra) < 20)
            {
                mission_info.setOpState(APOGEE);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(APOGEE);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                return APOGEE;
            }

            // Check last 3 readings are decreasing
            float r0 = alt_data.buffer[(idx + size - 2) % size];
            float r1 = alt_data.buffer[(idx + size - 1) % size];
            float r2 = alt_data.buffer[idx];

            if(r0 > r1 && r1 > r2 && (r0 - r2) > 10)
            {
                mission_info.setOpState(DESCENT);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(DESCENT);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                m_servo_camera.attach(SERVO_CAMERA_PIN);
                m_servo_gyro_left.attach(SERVO_GYRO_LEFT_PIN);
                m_servo_gyro_right.attach(SERVO_GYRO_RIGHT_PIN);
                return DESCENT;
            }
            break;
        }

        case APOGEE: {
            // Just check last 3 readings are decreasing
            float r0 = alt_data.buffer[(idx + size - 2) % size];
            float r1 = alt_data.buffer[(idx + size - 1) % size];
            float r2 = alt_data.buffer[idx];

            if(r0 > r1 && r1 > r2 && (r0 - r2) > 10)
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

        case DESCENT: {
            // Average of last 5 readings are 75% of max

            float alt_sum = 0.0;

            for (int i = 0; i < 3; i++)
            {
                int index = (idx - i + size) % size;
                alt_sum += alt_data.buffer[index];
            }
            alt_sum = alt_sum / 3;

            if (alt_sum <= 0.75 * alt_data.max_alt)
            {
                writeReleaseServo(47);
                mission_info.setOpState(PROBE_RELEASE);
                mission_info.beginPref("xb-set", false);
                int state_int = static_cast<int>(PROBE_RELEASE);
                mission_info.putPrefInt("opstate", state_int);
                mission_info.endPref();
                m_servo_camera.attach(SERVO_CAMERA_PIN);
                m_servo_gyro_left.attach(SERVO_GYRO_LEFT_PIN);
                m_servo_gyro_right.attach(SERVO_GYRO_RIGHT_PIN);
                writeGyroServoLeft(90);
                writeGyroServoRight(90);
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

    if(mission_info.getOpMode() == OPMODE_FLIGHT)
    {
        alt_data.buffer[alt_data.idx] = send_packet.ALTITUDE;
        alt_data.idx = (alt_data.idx + 1) % alt_data.window_size;
        alt_data.sample_count++;
    }

    // STATE
    strcpy(send_packet.STATE, op_state_to_string(updateState(mission_info.getOpState(), mission_info, ser)));

    // TEAM ID
    send_packet.TEAM_ID_PCKT = TEAM_ID;

    // MODE
    strcpy(send_packet.MODE, op_mode_to_string(mission_info.getOpMode(), 0));

    getRtcTime(send_packet.MISSION_TIME);
    send_packet.TEMPERATURE = getTemp();
    
    send_packet.VOLTAGE = getVoltage();

    // Magnetometer
    sensors_event_t event_lis3mdl;
    lis3mdl.getEvent(&event_lis3mdl);
    mx = event_lis3mdl.magnetic.x;
    my = event_lis3mdl.magnetic.y;
    mz = event_lis3mdl.magnetic.z;

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

    // Hall Effect Sensor
    if(mission_info.getOpState() == PROBE_RELEASE)
    {
        send_packet.AUTO_GYRO_ROTATION_RATE = getRotRate();
    }
    else
    {
        send_packet.AUTO_GYRO_ROTATION_RATE = 0.0;
    }

    // GPS
    int gps_avail = GPS_Serial->available();
    String gpsData = "";
    for (int i = 0; i < gps_avail; i++)
    {
        gpsData += (char)GPS_Serial->read();
    }

    if (gpsData.length() > 0)
    {
        // ser.sendInfoDataMsg("gpsData: %s", gpsData);
        for (size_t i = 0; i < gpsData.length(); i++)
        {
            gps.encode(gpsData[i]);
        }
    }

    if (gps.location.isUpdated() || 
        gps.time.isUpdated() ||
        gps.altitude.isUpdated() ||
        gps.satellites.isUpdated())
    {
        getGpsLat(&send_packet.GPS_LATITUDE);
        getGpsLong(&send_packet.GPS_LONGITUDE);
        getGpsTime(send_packet.GPS_TIME);
        getGpsAlt(&send_packet.GPS_ALTITUDE, mission_info.getLaunchAlt());
        getGpsSats(&send_packet.GPS_SATS);
    }

    // CAMERA status
    int cam1_state = getCamera1Status();
    int cam2_state = getCamera2Status();

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

    // Timer between camera servo movements
    if(mission_info.getOpState() == PROBE_RELEASE)
    {
        unsigned long now = millis();
        float dt = (now - last_camera_servo_update) / 1000.0;
        last_camera_servo_update = now;
        
        float magYaw = pid_cntl.computeTiltCompensatedYaw(mx, my, mz, ax, ay, az);
        pid_cntl.kalmanUpdate(gyroZ, magYaw, dt);
        updateCameraServo();
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
        writeReleaseServo(47);
        ser.sendInfoMsg("Waiting 10 seconds for release servo setup...");
        delay(10000);
        writeReleaseServo(0);
    }
    delay(100);
    ser.sendInfoMsg("Initialized release servo...");

    // GPS
    GPS_Serial->begin(9600, SERIAL_8N1, RX0_PIN, TX0_PIN);
    delay(100);
    ser.sendInfoMsg("Initialized GPS...");
    
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
    ser.sendInfoMsg("Initialized camera pins...");

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

    last_camera_servo_update = millis();

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
    m_servo_gyro_right.write(pos+15);
}

void SensorManager::writeGyroServoLeft(int pos)
{
    m_servo_gyro_left.write(pos+6);
}

void SensorManager::writeCameraServo(int pos)
{
    m_servo_camera.write(pos);
}

void SensorManager::getGpsAlt(float *alt, float launch_alt)
{
    if (gps.altitude.isValid())
    {
        *alt = gps.altitude.meters() - launch_alt;
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

int SensorManager::getCamera1Status()
{
    return digitalRead(CAMERA1_STATUS_PIN);
}

int SensorManager::getCamera2Status()
{
    return digitalRead(CAMERA2_STATUS_PIN);
}

void SensorManager::updateCameraServo() 
{
    float angle_to_north = fmod((360.0 - pid_cntl.getYawEstimate()), 360.0);
    
    float delta = angle_to_north - 180.0;
    if(delta > 180.0f) 
    {
        delta -= 360.0f;
    }
    
    int servo_movement = static_cast<int>(-1 * delta + 90.0);
    if(servo_movement <40) 
    {
        servo_movement = 40;
    }
    if(servo_movement > 140)
    {
        servo_movement = 140;
    }

    writeCameraServo(servo_movement);
}

float SensorManager::getAx()
{
    return ax;
}
float SensorManager::getAy()
{
    return ay;
}
float SensorManager::getAz()
{
    return az;
}
float SensorManager::getMx()
{
    return mx;
}
float SensorManager::getMy()
{
    return my;
}
float SensorManager::getMz()
{
    return mz;
}
float SensorManager::getGyroZ()
{
    return gyroZ;
}

void SensorManager::update_imu()
{
    sensors_event_t event_lis3mdl;
    lis3mdl.getEvent(&event_lis3mdl);
    mx = event_lis3mdl.magnetic.x;
    my = event_lis3mdl.magnetic.y;
    mz = event_lis3mdl.magnetic.z;

    // IMU
    for(int i = 0; i < 2; i++)
    {
        sh2_SensorValue_t sensorValue;
        if(bno08x->getSensorEvent(&sensorValue))
        {
            switch(sensorValue.sensorId)
            {
                case SH2_ACCELEROMETER:
                    ax = sensorValue.un.accelerometer.x;
                    ay = sensorValue.un.accelerometer.y;
                    az = sensorValue.un.accelerometer.z;
                    break;
                
                case SH2_GYROSCOPE_CALIBRATED:
                    gyroZ = sensorValue.un.gyroscope.z;
                    break;
                
                    
            }
        }
    }
}