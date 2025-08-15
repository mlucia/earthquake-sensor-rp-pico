<<<<<<< HEAD:legacy/main.py
# Version 1.3.0
=======
# Version 0.10
>>>>>>> 743414e (reorg - again):main.py
from machine import I2C, Pin, UART, RTC, Timer, SPI
from utime import sleep_us, ticks_us, localtime
import math
import struct
from ili9341 import ILI9341, color565
from framebuf import FrameBuffer, RGB565
import glcdfont

<<<<<<< HEAD:legacy/main.py
VERSION = "1.3.0"

i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
spi = SPI(0, baudrate=30000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3))
=======
VERSION = "0.10"

i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
spi = SPI(0, baudrate=20000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3))
>>>>>>> 743414e (reorg - again):main.py
cs = Pin(5, Pin.OUT, value=1)
dc = Pin(13, Pin.OUT)
rst = Pin(14, Pin.OUT)
WIDTH = 320
HEIGHT = 240
display = ILI9341(spi, cs=cs, dc=dc, rst=rst, width=WIDTH, height=HEIGHT, rotation=1, font=glcdfont)

uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
uart.write(f"ADXL345 Sensor v{VERSION} Started\n".encode())

ADXL345_ADDR = 0x53
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
BW_RATE = 0x2C
DATAX0 = 0x32
ADS1115_ADDR = 0x48

TOP_WINDOW_Y = 0
TOP_WINDOW_HEIGHT = 120
BOTTOM_WINDOW_Y = 120
BOTTOM_WINDOW_HEIGHT = 120
GRAPH_Y_START = BOTTOM_WINDOW_Y + BOTTOM_WINDOW_HEIGHT - 60

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

latest_reading = None
magnitudes = []
max_magnitude = 0.0
readings = []
kalman_x = KalmanFilter()
kalman_y = KalmanFilter()
kalman_z = KalmanFilter()
rtc = RTC()

def adxl345_init():
    try:
        i2c0.writeto_mem(ADXL345_ADDR, POWER_CTL, bytes([0x08]))
        i2c0.writeto_mem(ADXL345_ADDR, DATA_FORMAT, bytes([0x00 | 0x08]))
        i2c0.writeto_mem(ADXL345_ADDR, BW_RATE, bytes([0x09]))  # 100 Hz
    except Exception as e:
        display.fill(0)
<<<<<<< HEAD:legacy/main.py
        display.text("ADXL345 Error", 0, 0, color565(255, 255, 255))
        display.text(str(e), 0, 16, color565(255, 255, 255))
=======
        display.text("ADXL345 Error", 0, 0, color565(255, 255, 255), use_font=False)
        display.text(str(e), 0, 20, color565(255, 255, 255), use_font=False)
>>>>>>> 743414e (reorg - again):main.py
        display.show()
        return False
    return True

def ads1115_init():
    try:
        if ADS1115_ADDR in i2c0.scan():
            return True
        else:
            display.fill(0)
<<<<<<< HEAD:legacy/main.py
            display.text("ADS1115 Not Found", 0, 0, color565(255, 255, 255))
=======
            display.text("ADS1115 Not Found", 0, 0, color565(255, 255, 255), use_font=False)
>>>>>>> 743414e (reorg - again):main.py
            display.show()
            return False
    except Exception as e:
        display.fill(0)
<<<<<<< HEAD:legacy/main.py
        display.text("ADS1115 Error", 0, 0, color565(255, 255, 255))
        display.text(str(e), 0, 16, color565(255, 255, 255))
=======
        display.text("ADS1115 Error", 0, 0, color565(255, 255, 255), use_font=False)
        display.text(str(e), 0, 20, color565(255, 255, 255), use_font=False)
>>>>>>> 743414e (reorg - again):main.py
        display.show()
        return False

def read_ads1115():
    try:
<<<<<<< HEAD:legacy/main.py
        config = bytes([0x01, 0xC1, 0xE3])  # A0, 6.144V, 128 sps
        i2c0.writeto_mem(ADS1115_ADDR, 0x01, config)
        sleep_us(1000)  # ~7.8 ms conversion time
        data = i2c0.readfrom_mem(ADS1115_ADDR, 0x00, 2)
        raw = (data[0] << 8) | data[1]
        if raw & 0x8000:
            raw -= 0x10000
        voltage = (raw / 32768) * 6.144
        return voltage
    except:
        return 0.0
=======
        config = bytes([0x01, 0xC1, 0x83])
        i2c0.writeto_mem(ADS1115_ADDR, 0x01, config)
        sleep_us(1000)
        data = i2c0.readfrom_mem(ADS1115_ADDR, 0x00, 2)
        value = (data[0] << 8) | data[1]
        return value
    except:
        return 0
>>>>>>> 743414e (reorg - again):main.py

def read_accel():
    try:
        data = i2c0.readfrom_mem(ADXL345_ADDR, DATAX0, 6)
        a_x = (data[1] << 8 | data[0]) if (data[1] & 0x80) == 0 else (data[1] << 8 | data[0]) - 0x10000
        a_y = (data[3] << 8 | data[2]) if (data[3] & 0x80) == 0 else (data[3] << 8 | data[2]) - 0x10000
        a_z = (data[5] << 8 | data[4]) if (data[5] & 0x80) == 0 else (data[5] << 8 | data[4]) - 0x10000
        return a_x, a_y, a_z
    except Exception:
        return 0, 0, 0

def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 256.0
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

def draw_magnitude_graph(magnitudes):
<<<<<<< HEAD:legacy/main.py
    if not magnitudes:
        return
    max_mag = max(magnitudes)
    scale_factor = 60 / max(max_mag, 0.1)
    for i in range(min(len(magnitudes), WIDTH)):
        height = int(magnitudes[i] * scale_factor)
        height = max(1, min(60, height))
        display.fill_rect(i, HEIGHT - height, 1, height, color565(0, 255, 0))

def draw_borders():
    display.rect(0, TOP_WINDOW_Y, WIDTH, TOP_WINDOW_HEIGHT, color565(255, 255, 255))
    display.rect(0, BOTTOM_WINDOW_Y, WIDTH, BOTTOM_WINDOW_HEIGHT, color565(255, 255, 255))

def create_miniseed_record(timestamp_ms, a_x, a_y, a_z):
    header = bytearray(32)
    header[0:6] = b'000001'  # Sequence number
    header[6:8] = b'D '       # Quality indicator
    header[8:12] = b'PICO'    # Station ID
    header[12:16] = b'LHZ '   # Channel
    header[16:20] = b'XX00'   # Network code
    year, month, day, hour, minute, second = localtime(timestamp_ms // 1000)[:6]
    header[20:22] = struct.pack('>H', year)
    header[22:24] = struct.pack('>H', (month * 100 + day))
    header[24:27] = bytes([hour, minute, second])
    header[30:32] = struct.pack('>H', 100)  # Sample rate
    data = struct.pack('>hhh', int(a_x * 1000), int(a_y * 1000), int(a_z * 1000))
    return header + data

=======
    if len(magnitudes) > 0:
        max_mag = max(magnitudes)
        scale_factor = 60 / max(max_mag, 0.1)
        for i in range(min(len(magnitudes), WIDTH)):
            height = int(magnitudes[i] * scale_factor)
            height = min(height, 60)
            display.vline(i, HEIGHT - height - 1, height, color565(0, 255, 0))

def draw_borders():
    display.rect(0, TOP_WINDOW_Y, WIDTH, TOP_WINDOW_HEIGHT, color565(255, 255, 255))
    display.rect(0, BOTTOM_WINDOW_Y, WIDTH, BOTTOM_WINDOW_HEIGHT, color565(255, 255, 255))

>>>>>>> 743414e (reorg - again):main.py
def timer_callback(timer):
    global latest_reading, magnitudes, max_magnitude, readings
    start_time_us = ticks_us()
    start_time = localtime()

    raw_x, raw_y, raw_z = read_accel()
    if raw_x == 0 and raw_y == 0 and raw_z == 0:
        return

    a_x_raw, a_y_raw, a_z_raw = convert_to_g(raw_x, raw_y, raw_z)
    a_x_filt = kalman_x.update(a_x_raw)
    a_y_filt = kalman_y.update(a_y_raw)
    a_z_filt = kalman_z.update(a_z_raw - 1.0)
    magnitude_filt = math.sqrt(a_x_filt**2 + a_y_filt**2 + a_z_filt**2)

    max_magnitude = max(max_magnitude, magnitude_filt)
    magnitudes.append(magnitude_filt)
    if len(magnitudes) > WIDTH:
        magnitudes.pop(0)

    timestamp_ms = (start_time[3] * 3600 + start_time[4] * 60 + start_time[5]) * 1000
    timestamp_str = f"{start_time[3]:02d}:{start_time[4]:02d}:{start_time[5]:02d}"
    latest_reading = {
        'timestamp_ms': timestamp_ms,
        'a_x_raw': a_x_raw,
        'a_y_raw': a_y_raw,
        'a_z_raw': a_z_raw,
        'a_x_filt': a_x_filt,
        'a_y_filt': a_y_filt,
        'a_z_filt': a_z_filt,
        'magnitude_filt': magnitude_filt,
        'timestamp_str': timestamp_str,
        'voltage': 0.0  # Updated in main loop
    }
    readings.append({
        'timestamp': timestamp_str,
        'magnitude': magnitude_filt
    })
    if len(readings) > 20:
        readings.pop(0)

display.fill(0)
<<<<<<< HEAD:legacy/main.py
display.text(f"Version {VERSION}", 80, 100, color565(255, 255, 255))
=======
display.text(f"Version {VERSION}", 80, 96, color565(255, 255, 255), use_font=True)
>>>>>>> 743414e (reorg - again):main.py
display.show()
sleep_us(2000000)

if not adxl345_init():
    while True:
        sleep_us(1000000)

if not ads1115_init():
    while True:
        sleep_us(1000000)

<<<<<<< HEAD:legacy/main.py
rtc.datetime((2025, 6, 20, 20, 41, 0, 0, 0))  # June 20, 2025, 01:41 PM MST
=======
rtc.datetime((2025, 6, 19, 20, 25, 0, 0, 0))  # June 19, 2025, 01:25 PM MST (20:25 UTC)
>>>>>>> 743414e (reorg - again):main.py

timer = Timer()
timer.init(period=12, mode=Timer.PERIODIC, callback=timer_callback)  # 100 Hz with margin

<<<<<<< HEAD:legacy/main.py
sample_interval_us = 12000
=======
sample_interval_us = 2000
>>>>>>> 743414e (reorg - again):main.py
while True:
    start_time_us = ticks_us()

    if latest_reading:
<<<<<<< HEAD:legacy/main.py
        latest_reading['voltage'] = read_ads1115()
        display.fill(0)
        display.text("Network Disabled", 8, TOP_WINDOW_Y + 8, color565(255, 255, 255))
        raw_str = f"X:{latest_reading['a_x_raw']:.2f} Y:{latest_reading['a_y_raw']:.2f} Z:{latest_reading['a_z_raw']:.2f}"
        display.text(raw_str, 8, BOTTOM_WINDOW_Y + 8, color565(255, 255, 255))
        display.text(f"A0: {latest_reading['voltage']:.3f}V", 8, BOTTOM_WINDOW_Y + 24, color565(255, 255, 0))
=======
        display.fill(0)
        display.text("Network Disabled", 8, TOP_WINDOW_Y + 8, color565(255, 255, 255), use_font=True)
        raw_str = f"X:{latest_reading['a_x_raw']:.2f} Y:{latest_reading['a_y_raw']:.2f} Z:{latest_reading['a_z_raw']:.2f}"
        display.text(raw_str, 8, BOTTOM_WINDOW_Y + 8, color565(255, 255, 255), use_font=True)
>>>>>>> 743414e (reorg - again):main.py
        draw_magnitude_graph(magnitudes)
        draw_borders()
        display.show()

<<<<<<< HEAD:legacy/main.py
        miniseed = create_miniseed_record(
=======
        output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
>>>>>>> 743414e (reorg - again):main.py
            latest_reading['timestamp_ms'],
            latest_reading['a_x_filt'],
            latest_reading['a_y_filt'],
            latest_reading['a_z_filt']
        )
        uart.write(miniseed)

<<<<<<< HEAD:legacy/main.py
        elapsed_us = ticks_us() - start_time_us
        uart.write(f"Frame time: {elapsed_us/1000:.1f} ms\n".encode())

    sleep_us(max(0, sample_interval_us - (ticks_us() - start_time_us)))
=======
    elapsed_us = ticks_us() - start_time_us
    sleep_us(max(0, sample_interval_us - elapsed_us))
>>>>>>> 743414e (reorg - again):main.py
