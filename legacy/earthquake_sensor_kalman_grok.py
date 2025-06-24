from machine import I2C, Pin, UART
from os import listdir, chdir
from utime import ticks_ms, sleep_us
import math

# I2C setup for MPU6050
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # I2C0 on GP1 (SCL), GP0 (SDA)

# UART setup for output (TX on GP4, 115200 baud)
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"MPU6050 Earthquake Sensor with Kalman Filter Started\n")
print("MPU6050 Earthquake Sensor with Kalman Filter Started\n")


# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
CONFIG = 0x1A
SMPLRT_DIV = 0x19

# Kalman Filter Class for each axis
class KalmanFilter:
    def __init__(self):
        self.x = 0.0  # Estimated state (acceleration)
        self.p = 1.0  # State covariance
        self.q = 0.001  # Process noise covariance
        self.r = 0.0004  # Measurement noise covariance (from 400 µg/√Hz at 500 Hz)
        self.k = 0.0  # Kalman gain

    def update(self, measurement):
        # Prediction step
        self.p = self.p + self.q
        
        # Update step
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p
        
        return self.x

# Initialize MPU6050 and Kalman filters
def mpu6050_init():
    i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, bytes([0x00]))  # Wake up
    i2c.writeto_mem(MPU6050_ADDR, ACCEL_CONFIG, bytes([0x00]))  # ±2g range
    i2c.writeto_mem(MPU6050_ADDR, CONFIG, bytes([0x03]))  # DLPF 44 Hz
    i2c.writeto_mem(MPU6050_ADDR, SMPLRT_DIV, bytes([0x01]))  # ~500 Hz

# Read raw accelerometer data
def read_accel():
    data = i2c.readfrom_mem(MPU6050_ADDR, ACCEL_XOUT_H, 6)
    a_x = (data[0] << 8 | data[1]) if (data[0] << 8 | data[1]) < 32768 else (data[0] << 8 | data[1]) - 65536
    a_y = (data[2] << 8 | data[3]) if (data[2] << 8 | data[3]) < 32768 else (data[2] << 8 | data[3]) - 65536
    a_z = (data[4] << 8 | data[5]) if (data[4] << 8 | data[5]) < 32768 else (data[4] << 8 | data[5]) - 65536
    return a_x, a_y, a_z

# Convert raw data to g
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 16384.0
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Main loop
mpu6050_init()
sample_interval_us = 2000  # 2 ms for 500 Hz
threshold_g = 0.1  # Seismic event threshold
kalman_x = KalmanFilter()
kalman_y = KalmanFilter()
kalman_z = KalmanFilter()

while True:
    start_time = ticks_ms()
    
    # Read and convert accelerometer data
    raw_x, raw_y, raw_z = read_accel()
    a_x_raw, a_y_raw, a_z_raw = convert_to_g(raw_x, raw_y, raw_z)
    
    # Apply Kalman filter
    a_x_filt = kalman_x.update(a_x_raw)
    a_y_filt = kalman_y.update(a_y_raw)
    a_z_filt = kalman_z.update(a_z_raw - 1.0)  # Adjust for gravity (~1g)
    
    # Calculate magnitude for threshold detection
    magnitude_raw = math.sqrt(a_x_raw**2 + a_y_raw**2 + a_z_raw**2)
    magnitude_filt = math.sqrt(a_x_filt**2 + a_y_filt**2 + a_z_filt**2)
    seismic_event = magnitude_raw > threshold_g or magnitude_filt > threshold_g
    
    # Format output: timestamp, raw_x, raw_y, raw_z, filt_x, filt_y, filt_z, seismic_flag
    output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:d}\n".format(
        start_time, a_x_raw, a_y_raw, a_z_raw, a_x_filt, a_y_filt, a_z_filt, 1 if seismic_event else 0
    )
    uart.write(output.encode())
    print(output + "\n")
    # Maintain 500 Hz sampling rate
    elapsed_us = (ticks_ms() - start_time) * 1000
    sleep_us(max(0, sample_interval_us - elapsed_us))