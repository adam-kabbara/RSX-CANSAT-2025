#include "sensorManager.h"

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
                return LAUNCH_PAD;
            }
            break;
        }

    }

    return curr_state;
}

void SensorManager::sampleSensors(MissionManager &mission_info)
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

    mission_info.setOpState(updateState(mission_info.getOpState(), mission_info));

    strcpy(send_packet.STATE, op_state_to_string(mission_info.getOpState()));

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

    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;

    getGyroData(&roll, &pitch, &yaw);

    send_packet.GYRO_R = roll;
    send_packet.GYRO_P = pitch;
    send_packet.GYRO_Y = yaw;

    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    getAccelData(&roll, &pitch, &yaw);

    send_packet.ACCEL_R = roll;
    send_packet.ACCEL_P = pitch;
    send_packet.ACCEL_Y = yaw;

    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;

    getMagData(&roll, &pitch, &yaw);

    send_packet.MAG_R = roll;
    send_packet.MAG_P = pitch;
    send_packet.MAG_Y = yaw;

    send_packet.AUTO_GYRO_ROTATION_RATE = getRotRate();

    getGpsTime(send_packet.GPS_TIME);
    send_packet.GPS_ALTITUDE = getGpsAlt();
    send_packet.GPS_LATITUDE = getGpsLat();
    send_packet.GPS_LONGITUDE = getGpsLong();
    send_packet.GPS_SATS = getGpsSats();

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
        "%d,%s,%d,%s,%s," //5
        "%.1f,%.1f,%.1f,%.1f," //4
        "%d,%d,%d,%d,%d," //5
        "%d,%d,%.1f,%.1f,%.1f," //5
        "%s,%.1f,%.4f," //3
        "%.4f,%d,%s,%d", //3
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

void SensorManager::startSensors(SerialManager &ser)
{
    // Temperature and Pressure
    uint8_t status = bme.begin(0x77);
    delay(100);
    
    // Release Servo
    m_servo_release.attach(SERVO_RELEASE_PIN);
    delay(1000);
    writeReleaseServo(38);
    ser.sendInfoMsg("Waiting 10 seconds for release servo setup...");
    delay(10000);
    writeReleaseServo(0);
    delay(100);
    
    // Gyro Servo 1
    m_servo_gyro_1.attach(SERVO_GYRO1_PIN);
    delay(1000);
    writeGyroServo1(90);
    delay(100);
    
    // Gyro Servo 2
    m_servo_gyro_2.attach(SERVO_GYRO2_PIN);
    delay(1000);
    writeGyroServo2(90);
    delay(100);
    
    // Camera Servo
    m_servo_camera.attach(SERVO_CAMERA_PIN);
    delay(1000);
    writeCameraServo(90);
    delay(100);
    
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
    
    ser.sendInfoMsg("All sensors and servos initialized successfully!");
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

void SensorManager::writeGyroServo1(int pos)
{
    m_servo_gyro_1.write(pos);
}

void SensorManager::writeGyroServo2(int pos)
{
    m_servo_gyro_2.write(pos);
}

void SensorManager::writeCameraServo(int pos)
{
    m_servo_camera.write(pos);
}

float SensorManager::getGpsAlt()
{
    return 0;
}

float SensorManager::getGpsLat()
{
    return 0;
}

float SensorManager::getGpsLong()
{
    return 0;
}

int SensorManager::getGpsSats()
{
    return 0;
}

void SensorManager::getRtcTime(char time_str[DATA_SIZE])
{
    strncpy(time_str, "00:00:00", DATA_SIZE - 1);
    time_str[DATA_SIZE - 1] = '\0';
}

void SensorManager::getGpsTime(char time_str[DATA_SIZE])
{
    strncpy(time_str, "00:00:00", DATA_SIZE - 1);
    time_str[DATA_SIZE - 1] = '\0';
}

float SensorManager::getVoltage()
{
    int adc_raw = analogRead(ADC_VOLTAGE_PIN);
    float adc_voltage = (adc_raw / 4096.0) * 3.3; // 3.3V reference voltage
    
    float real_voltage = adc_voltage * (R1 + R2) / R2;
    return real_voltage;
}

float SensorManager::getRotRate()
{
    return 0;
}


void SensorManager::getMagData(float *r, float *p, float *y)
{
    *r = 0.0;
    *p = 0.0;
    *y = 0.0;
}

void SensorManager::getGyroData(float *r, float *p, float *y)
{
    *r = 0.0;
    *p = 0.0;
    *y = 0.0;
}

void SensorManager::getAccelData(float *r, float *p, float *y)
{
    *r = 0.0;
    *p = 0.0;
    *y = 0.0;
}
