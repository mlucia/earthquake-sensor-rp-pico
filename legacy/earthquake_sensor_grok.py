from machine import I2C, Pin, UART
from utime import ticks_ms, sleep_us
import math

# I2C setup for MPU6050
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # I2C0 on GP1 (SCL), GP0 (SDA)

# UART setup for output (TX on GP4, 115200 baud)
uart = UART( 1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"MPU6050 Earthquake Sensor Started\n")

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
CONFIG = 0x1A
SMPLRT_DIV = 0x19

# Initialize MPU6050
def mpu6050_init():
    # Wake up MPU6050 (disable sleep mode)
    i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, bytes([0x00]))
    # Set accelerometer range to ±2g (0x00 = ±2g, sensitivity 16384 LSB/g)
    i2c.writeto_mem(MPU6050_ADDR, ACCEL_CONFIG, bytes([0x00]))
    # Set DLPF to 44 Hz (0x03) to filter out high-frequency noise
    i2c.writeto_mem(MPU6050_ADDR, CONFIG, bytes([0x03]))
    # Set sample rate divider: 1000 Hz / (1 + SMPLRT_DIV) = ~500 Hz
    # SMPLRT_DIV = 1 for ~500 Hz (1000 / (1 + 1) = 500 Hz)
    i2c.writeto_mem(MPU6050_ADDR, SMPLRT_DIV, bytes([0x01]))

# Read raw accelerometer data (6 bytes: XH, XL, YH, YL, ZH, ZL)
def read_accel():
    data = i2c.readfrom_mem(MPU6050_ADDR, ACCEL_XOUT_H, 6)
    # Combine high and low bytes, convert to signed 16-bit
    a_x = (data[0] << 8 | data[1]) if (data[0] << 8 | data[1]) < 32768 else (data[0] << 8 | data[1]) - 65536
    a_y = (data[2] << 8 | data[3]) if (data[2] << 8 | data[3]) < 32768 else (data[2] << 8 | data[3]) - 65536
    a_z = (data[4] << 8 | data[5]) if (data[4] << 8 | data[5]) < 32768 else (data[4] << 8 | data[5]) - 65536
    return a_x, a_y, a_z

# Convert raw accelerometer data to g (sensitivity: 16384 LSB/g for ±2g)
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 16384.0
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Main loop
mpu6050_init()
sample_interval_us = 2000  # 2000 µs = 2 ms for 500 Hz sampling
threshold_g = 0.1  # Threshold for detecting seismic events (0.1g)

while True:
    start_time = ticks_ms()
    
    # Read and convert accelerometer data
    raw_x, raw_y, raw_z = read_accel()
    a_x, a_y, a_z = convert_to_g(raw_x, raw_y, raw_z)
    
    # Calculate magnitude for threshold detection
    magnitude = math.sqrt(a_x**2 + a_y**2 + a_z**2)
    # Adjust a_z for gravity (approx 1g when stationary)
    a_z_adjusted = a_z - 1.0  # Assuming Z-axis is vertical, subtract 1g
    seismic_event = magnitude > threshold_g or abs(a_z_adjusted) > threshold_g
    
    # Format output: timestamp, a_x, a_y, a_z, seismic flag
    output = "{:d},{:.3f},{:.3f},{:.3f},{:d}\n".format(
        start_time, a_x, a_y, a_z, 1 if seismic_event else 0
    )
    uart.write(output.encode())
    
    # Maintain 500 Hz sampling rate
    elapsed_us = (ticks_ms() - start_time) * 1000
    sleep_us(max(0, sample_interval_us - elapsed_us))