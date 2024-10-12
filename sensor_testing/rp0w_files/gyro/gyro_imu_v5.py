import time
import board
import adafruit_mpu6050
from adafruit_extended_bus import ExtendedI2C as I2C

i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)

while True:
    try:
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(mpu.acceleration), end=" | ")
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro), end=" | ")
        print("Temperature: %.2f C"%mpu.temperature)
    except OSError:
        print("\nERROR CAUGHT: CLOCK STRETCHING FAILED\n")
    time.sleep(1)
