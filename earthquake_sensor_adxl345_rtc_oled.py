from machine import I2C, Pin, UART, RTC
from utime import sleep_us
import math
from ssd1306 import SSD1306_I2C  # Requires ssd1306.py library (e.g., from micropython-ssd1306)

# I2C setups
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # ADXL345 on I2C0
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)  # RTC and OLED on I2C1

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"ADXL345 Earthquake Sensor with RTC and OLED Started\n")

# ADXL345 Registers
ADXL345_ADDR = 0x53
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
BW_RATE = 0x2C
DATAX0 = 0x32

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
        self.r = 0.00015
        self.k = 0.0

    def update(self, measurement):
        self.p = self.p + self.q
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (0.5 - self.k) * self.p
        return self.x

# Initialize ADXL345
def adxl345_init():
    try:
        i2c0.writeto_mem(ADXL345_ADDR, POWER_CTL, bytes([0x08]))
        i2c0.writeto_mem(ADXL345_ADDR, DATA_FORMAT, bytes([0x00 | 0x08]))
        i2c0.writeto_mem(ADXL345_ADDR, BW_RATE, bytes([0x0A]))
    except Exception as e:
        oled.fill(0)
        oled.text("ADXL345 Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return False
    return True

# Initialize RTC (basic setup, assumes time is set externally or via code)
def rtc_init():
    try:
        # Example: Set time (adjust as needed, e.g., via NTP or manual input)
        rtc = machine.RTC()
        rtc.datetime((2025, 5, 17, 6, 12, 37, 0, 0))  # Year, Month, Day, Weekday, Hour, Minute, Second, Subsecond
    except Exception as e:
        oled.fill(0)
        oled.text("RTC Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return False
    return True

# Read ADXL345 data
def read_accel():
    try:
        data = i2c0.readfrom_mem(ADXL345_ADDR, DATAX0, 6)
        a_x = (data[1] << 8 | data[0]) if (data[1] & 0x80) == 0 else (data[1] << 8 | data[0]) - 0x10000
        a_y = (data[3] << 8 | data[2]) if (data[3] & 0x80) == 0 else (data[3] << 8 | data[2]) - 0x10000
        a_z = (data[5] << 8 | data[4]) if (data[5] & 0x80) == 0 else (data[5] << 8 | data[4]) - 0x10000
        return a_x, a_y, a_z
    except Exception as e:
        oled.fill(0)
        oled.text("Read Error", 0, 0)
        oled.text(str(e), 0, 10)
        oled.show()
        return 0, 0, 0

# Convert to g
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 256.0
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Graph seismic event
def draw_seismic_graph(magnitudes):
    oled.fill(0)
    oled.text("Seismic Event", 0, 0)
    if len(magnitudes) > 0:
        max_mag = max(magnitudes)
        for i in range(min(len(magnitudes), WIDTH)):
            height = int((magnitudes[i] / max(max_mag, 0.1)) * (HEIGHT - 20))  # Scale to 44 pixels
            oled.vline(i, HEIGHT - 10 - height, height, 1)
    oled.show()

# Main loop
if not adxl345_init() or not rtc_init():
    while True:
        sleep_us(1000000)  # Wait 1s on error
sample_interval_us = 2000
threshold_g = 0.1
kalman_x, kalman_y, kalman_z = KalmanFilter(), KalmanFilter(), KalmanFilter()
magnitudes = []  # Buffer for graph

while True:
    start_time = RTC().datetime()  # Use RTC for timestamp
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
    
    # Update magnitudes buffer for graph (last 64 samples)
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
    timestamp = (start_time[4] * 3600 + start_time[5] * 60 + start_time[6]) * 1000  # Convert to ms
    output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:d}\n".format(
        timestamp, a_x_raw, a_y_raw, a_z_raw, a_x_filt, a_y_filt, a_z_filt, 1 if seismic_event else 0
    )
    uart.write(output.encode())
    
    # Maintain 500 Hz
    sleep_us(max(0, sample_interval_us - (ticks_ms() - start_time[6] * 1000) * 1000))