from mpu6050 import mpu6050
import time
from time import sleep

imu = mpu6050(0x68)
t0 = time.time()
# Wait for MPU to Settle
settling_time = 5
print('Settling MPU for %d seconds' % settling_time)
time.sleep(4)
print('MPU is Done Settling')


def get_gyro():
    gyro_data = imu.get_gyro_data()
    gx = gyro_data['x']
    gy = gyro_data['y']
    gz = gyro_data['z']
    return gx, gy, gz
    

def gyro_calibration(calibration_time=10):
    """
        Description: This is a function to get the offset values
            for gyro calibration for mpu6050.
        
        Parameters:
        
        calibration_time[int]: Time in seconds you want to calibrate
            mpu6050. The longer the time the more accurate the
            calibration
    
        Outputs: Array with offsets pertaining to three axes of
            rotation [offset_gx, offset_gy, offset_gz]. Add these
            offsets to your sensor readins later on for more
            accurate readings!
    """
    print('--' * 25)
    print('Beginning Gyro Calibration - Do not move the MPU6050')
    
    # placeholder for the average of tuples in mpu_gyro_array
    offsets = [0, 0, 0]
    # placeholder for number of calculations we get from the mpu
    num_of_points = 0
    
    # We get the current time and add the calibration time
    end_loop_time = time.time() + calibration_time
    # We end the loop once the calibration time has passed
    while end_loop_time > time.time():
        num_of_points += 1
        (gx, gy, gz) = get_gyro()
        offsets[0] += gx
        offsets[1] += gy
        offsets[2] += gz
        
        # This is just to show you its still calibrating :)
        if num_of_points % 100 == 0:
            print('Still Calibrating Gyro... %d points so far' % num_of_points)
        
    print('Calibration for Gyro is Complete! %d points total' % num_of_points)
    offsets = [i/num_of_points for i in offsets] # we divide by the length to get the mean
    return offsets


if __name__ == "__main__":
    print(gyro_calibration(60))
