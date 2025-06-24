from machine import I2C, Pin, UART, RTC
from utime import sleep_us, ticks_ms
import math
from ssd1306 import SSD1306_I2C  # Requires ssd1306.py library

# I2C setups
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # LSM6DS3 on I2C0
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)  # RTC and OLED on I2C1

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"LSM6DS3 Earthquake Sensor with RTC and OLED Started\n")  # Updated message

# LSM6DS3 Registers
LSM6DS3_ADDR = 0x6A  # Changed from 0x68 (MPU6050) to 0x6A (SA0 low)
CTRL1_XL = 0x10      # Accelerometer control
CTRL3_C = 0x12       # General control
OUTX_L_XL = 0x28     # Accelerometer output start

# RTC and OLED constants (unchanged)
RTC_ADDR = 0x68
OLED_ADDR = 0x3C
WIDTH = 128
HEIGHT = 64
oled = SSD1306_I2C(WIDTH, HEIGHT, i2c1)

# Kalman Filter Class (unchanged)
class KalmanFilter:
    def __init__(self):
        self.x = 0.0
        self.p = 1.0
        self.q = 0.001
        self.r = 0.0004  # LSM6DS3 noise similar (~400 µg/√Hz at 833 Hz)
        self.k = 0.0

    def update(self, measurement):
        self.p = self.p + self.q
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (0.5 - self.k) * self.p
        return self.x

# Initialize LSM6DS3
def lsm6ds3_init():
    try:
        # Enable block data update and auto-increment
        i2c0.writeto_mem(LSM6DS3_ADDR, CTRL3_C, bytes([0x44]))  # BDU=1, IF_INC=1
        # Accelerometer: 833 Hz, ±2g (0x60 = ODR 833 Hz, FS ±2g)
        i2c0.writeto_mem(LSM6DS3_ADDR, CTRL1_XL, bytes([0x60]))
        return True
    except Exception as e:
        oled.fill(0)
        oled.text("LSM6DS3 Error", 0, 0)  # Updated error message
        oled.text(str(e), 0, 10)
        oled.show()
        return False

# Initialize RTC (unchanged)
def rtc_init():
    try:
        rtc = machine.RTC()
        rtc.datetime((2025, 5, 21, 6, 12, 42, 0, 0))  # Same initial time
    except Exception as e:
        oled.fill(0)
        oled.text("RTC Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return False
    return True

# Read LSM6DS3 accelerometer data
def read_accel():
    try:
        data = i2c0.readfrom_mem(LSM6DS3_ADDR, OUTX_L_XL, 6)
        # LSM6DS3: Two's complement, little-endian
        a_x = (data[1] << 8 | data[0]) if (data[1] << 8 | data[0]) < 32768 else (data[1] << 8 | data[0]) - 65536
        a_y = (data[3] << 8 | data[2]) if (data[3] << 8 | data[2]) < 32768 else (data[3] << 8 | data[2]) - 65536
        a_z = (data[5] << 8 | data[4]) if (data[5] << 8 | data[4]) < 32768 else (data[5] << 8 | data[4]) - 65536
        return a_x, a_y, a_z
    except Exception as e:
        oled.fill(0)
        oled.text("Read Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return 0, 0, 0

# Convert to g
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 16393.0  # LSM6DS3 ±2g: 0.061 mg/LSB ≈ 16393 LSB/g
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Draw seismic graph (unchanged)
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
if not lsm6ds3_init() or not rtc_init():  # Updated to lsm6ds3_init
    while True:
        sleep_us(1000000)
sample_interval_us = 2000  # Maintain 500 Hz
threshold_g = 2
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
    a_z_filt = kalman_z.update(a_z_raw - 1.0)  # Subtract 1g for z-axis
    magnitude_raw = math.sqrt(a_x_raw**2 + a_y_raw**2 + a_z_raw**2)
    magnitude_filt = math.sqrt(a_x_filt**2 + a_y_filt**2 + a_z_filt**2)
    seismic_event = magnitude_raw > threshold_g or magnitude_filt > threshold_g
    
    # Update magnitudes buffer (unchanged)
    magnitudes.append(magnitude_filt)
    if len(magnitudes) > WIDTH:
        magnitudes.pop(0)
    
    # OLED status (unchanged)
    oled.fill(0)
    oled.text("Status: Running", 0, 0)
    oled.text(f"Mag: {magnitude_filt:.2f}g", 0, 10)
    oled.text(f"Event: {1 if seismic_event else 0}", 0, 20)
    if seismic_event:
        draw_seismic_graph(magnitudes)
    else:
        oled.text("No Event", 0, 30)
    oled.show()
    
    # UART output (unchanged)
    timestamp = (start_time[4] * 3600 + start_time[5] * 60 + start_time[6]) * 1000
    output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:d}\n".format(
        timestamp, a_x_raw, a_y_raw, a_z_raw, a_x_filt, a_y_filt, a_z_filt, 1 if seismic_event else 0
    )
    uart.write(output.encode())
    
    # Maintain 500 Hz (unchanged)
    sleep_us(max(0, sample_interval_us - (ticks_ms() - start_time[6] * 1000) * 1000))
    