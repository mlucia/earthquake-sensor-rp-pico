from machine import I2C, Pin, UART, RTC, reset
from utime import sleep_us, ticks_us
import math
import network
import socket
import urequests
from ssd1306 import SSD1306_I2C

# WiFi credentials
SSID = "Your_WiFi_SSID"
PASSWORD = "Your_WiFi_Password"

# OTA configuration
OTA_GITHUB_URL = "https://raw.githubusercontent.com/your_username/your_repo/main/main.py"
OTA_CHECK_INTERVAL = 60000000  # Check every 60 seconds (in microseconds)
LAST_OTA_CHECK = 0

# I2C setups
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"ADXL345 Earthquake Sensor with OTA Started\n")

# On-board LED
led = Pin(25, Pin.OUT)
led_state = False

# ADXL345 Registers
ADXL345_ADDR = 0x53
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
BW_RATE = 0x2C
DATAX0 = 0x32

# RTC and OLED constants
RTC_ADDR = 0x68
OLED_ADDR = 0x3C
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

# Initialize RTC
rtc = RTC()
def rtc_init():
    try:
        rtc.datetime((2025, 6, 9, 1, 16, 23, 0, 0))
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
            height = int((magnitudes[i] / max(max_mag, 0.1)) * (HEIGHT - 20))
            oled.vline(i, HEIGHT - 10 - height, height, 1)
    oled.show()

# WiFi connection
def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        print('Waiting for WiFi connection...')
        sleep_us(1000000)
    if wlan.status() != 3:
        oled.fill(0)
        oled.text("WiFi Failed", 0, 0)
        oled.show()
        raise RuntimeError('WiFi connection failed')
    else:
        ip = wlan.ifconfig