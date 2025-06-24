from machine import I2C, Pin, UART, RTC
from utime import sleep_us, ticks_ms
import math
from ssd1306 import SSD1306_I2C  # Requires ssd1306.py library

# I2C setups
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # MPU6050 on I2C0
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)  # RTC and OLED on I2C1

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"MPU6050 Earthquake Sensor with RTC and OLED Started\n")

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
CONFIG = 0x1A
SMPLRT_DIV = 0x19
ACCEL_XOUT_H = 0x3B

# RTC and OLED constants
RTC_ADDR = 0x68
OLED_ADDR = 0x3C  # Adjust to 0x3D if needed
WIDTH = 128
HEIGHT = 64
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c1)

# Kalman Filter Class
class KalmanFilter:
    def __init__(self):
        self.x = 0.0
        self.p = 1.0
        self.q = 0.001
        self.r = 0.0004  # MPU6050 noise (~400 µg/√Hz at 500 Hz)
        self.k = 0.0

    def update(self, measurement):
        self.p = self.p + self.q
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (0.5 - self.k) * self.p
        return self.x

# Initialize MPU6050
def mpu6050_init():
    try:
        i2c0.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, bytes([0x00]))  # Wake up
        i2c0.writeto_mem(MPU6050_ADDR, ACCEL_CONFIG, bytes([0x00]))  # ±2g range
        i2c0.writeto_mem(MPU6050_ADDR, CONFIG, bytes([0x03]))  # DLPF 44 Hz
        i2c0.writeto_mem(MPU6050_ADDR, SMPLRT_DIV, bytes([0x01]))  # ~500 Hz (1000 / (1 + 1))
    except Exception as e:
        oled.fill(0)
        oled.text("MPU6050 Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return False
    return True

# Initialize RTC
def rtc_init():
    try:
        rtc = machine.RTC()
        rtc.datetime((2025, 5, 21, 6, 12, 42, 0, 0))  # Set to 12:42 PM MST, May 17, 2025
    except Exception as e:
        oled.fill(0)
        oled.text("RTC Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return False
    return True

# Read MPU6050 data
def read_accel():
    try:
        data = i2c0.readfrom_mem(MPU6050_ADDR, ACCEL_XOUT_H, 6)
        a_x = (data[0] << 8 | data[1]) if (data[0] << 8 | data[1]) < 32768 else (data[0] << 8 | data[1]) - 65536
        a_y = (data[2] << 8 | data[3]) if (data[2] << 8 | data[3]) < 32768 else (data[2] << 8 | data[3]) - 65536
        a_z = (data[4] << 8 | data[5]) if (data[4] << 8 | data[5]) < 32768 else (data[4] << 8 | data[5]) - 65536
        return a_x, a_y, a_z
    except Exception as e:
        oled.fill(0)
        oled.text("Read Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return 0, 0, 0

# Convert to g
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 16384.0  # ±2g range
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Draw seismic graph
def draw_seismic_graph(magnitudes):
    oled.fill(0)
    oled.text("Seismic Event", 0, 0)
    if len(magnitudes) > 0:
        max_mag = max(magnitudes)
        for i in range(min(len(magnitudes), WIDTH)):
            height = int((magnitudes[i] / max(max_mag, 0.1)) * (HEIGHT - 20))
            oled.vline(i, HEIGHT - 10 - height, height, 1)
    oled.show()

# Main loop
if not mpu6050_init() or not rtc_init():
    while True:
        sleep_us(1000000)  # Wait 1s on error
sample_interval_us = 2000
threshold_g = 1
kalman_x, kalman_y, kalman_z = KalmanFilter(), KalmanFilter(), KalmanFilter()
magnitudes = []

while True:
    start_time = RTC().datetime()
    raw_x, raw_y, raw_z = read_accel()
    if raw_x == 0 and raw_y == 0 and raw_z == 0:
        continue
    a_x_raw, a_y_raw, a_z_raw = convert_to_g(raw_x, raw_y, raw_z)
    a_x_filt = kalman_x.update(a_x_raw)
    a_y_filt = kalman_y.update(a_y_raw)
    a_z_filt = kalman_z.update(a_z_raw - 1.0)
    magnitude_raw = math.sqrt(a_x_raw**2 + a_y_raw**2 + a_z_raw**2)
    magnitude_filt = math.sqrt(a_x_filt**2 + a_y_filt**2 + a_z_filt**2)
    seismic_event = magnitude_raw > threshold_g or magnitude_filt > threshold_g
    
    # Update magnitudes buffer
    magnitudes.append(magnitude_filt)
    if len(magnitudes) > WIDTH:
        magnitudes.pop(0)
    
    # OLED status
    oled.fill(0)
    oled.text("Status: Running", 0, 0)
    oled.text(f"Mag: {magnitude_filt:.2f}g", 0, 10)
    oled.text(f"Event: {1 if seismic_event else 0}", 0, 20)
    if seismic_event:
        draw_seismic_graph(magnitudes)
    else:
        oled.text("No Event", 0, 30)
    oled.show()
    
    # UART output
    timestamp = (start_time[4] * 3600 + start_time[5] * 60 + start_time[6]) * 1000
    output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:d}\n".format(
        timestamp, a_x_raw, a_y_raw, a_z_raw, a_x_filt, a_y_filt, a_z_filt, 1 if seismic_event else 0
    )
    uart.write(output.encode())
    
    # Maintain 500 Hz
    sleep_us(max(0, sample_interval_us - (ticks_ms() - start_time[6] * 1000) * 1000))