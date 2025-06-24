from machine import I2C, Pin, UART
from utime import ticks_ms, sleep_us
import math

# I2C setup for ADXL345
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # I2C0 on GP1 (SCL), GP0 (SDA)

# UART setup for output (TX on GP4, 115200 baud)
uart = UART(0, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"ADXL345 Earthquake Sensor with Kalman Filter Started\n")

# ADXL345 Registers
ADXL345_ADDR = 0x53  # I2C address (AD0 low)
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
BW_RATE = 0x2C
DATAX0 = 0x32  # Start of 6-byte acceleration data

# Kalman Filter Class for each axis
class KalmanFilter:
    def __init__(self):
        self.x = 0.0  # Estimated state (acceleration)
        self.p = 1.0  # State covariance
        self.q = 0.001  # Process noise covariance
        self.r = 0.00015  # Measurement noise covariance (~150 µg/√Hz at 500 Hz)
        self.k = 0.0  # Kalman gain

    def update(self, measurement):
        # Prediction step
        self.p = self.p + self.q
        
        # Update step
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (0.5 - self.k) * self.p  # Adjusted for stability
        
        return self.x

# Initialize ADXL345
def adxl345_init():
    # Wake up and enable measurements
    i2c.writeto_mem(ADXL345_ADDR, POWER_CTL, bytes([0x08]))  # Measurement mode
    # Set range to ±2g (0x00), full resolution
    i2c.writeto_mem(ADXL345_ADDR, DATA_FORMAT, bytes([0x00 | 0x08]))  # 0x08 for full resolution
    # Set bandwidth to ~50 Hz (0x0A = 100 Hz ODR, internal filter ~50 Hz)
    i2c.writeto_mem(ADXL345_ADDR, BW_RATE, bytes([0x0A]))

# Read raw accelerometer data (6 bytes: X0, X1, Y0, Y1, Z0, Z1)
def read_accel():
    data = i2c.readfrom_mem(ADXL345_ADDR, DATAX0, 6)
    # Combine bytes, convert to signed 16-bit (shift and mask)
    a_x = (data[1] << 8 | data[0]) if (data[1] & 0x80) == 0 else (data[1] << 8 | data[0]) - 0x10000
    a_y = (data[3] << 8 | data[2]) if (data[3] & 0x80) == 0 else (data[3] << 8 | data[2]) - 0x10000
    a_z = (data[5] << 8 | data[4]) if (data[5] & 0x80) == 0 else (data[5] << 8 | data[4]) - 0x10000
    return a_x, a_y, a_z

# Convert raw data to g (sensitivity: 256 LSB/g for ±2g)
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 256.0
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Main loop
adxl345_init()
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
    
    # Maintain 500 Hz sampling rate
    elapsed_us = (ticks_ms() - start_time) * 1000
    sleep_us(max(0, sample_interval_us - elapsed_us))