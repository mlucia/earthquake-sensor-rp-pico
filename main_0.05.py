from machine import I2C, Pin, UART, RTC, Timer, SPI
from utime import sleep_us, ticks_us, localtime
import math
from ili9341 import ILI9341, color565
from framebuf import FrameBuffer, RGB565

# Version number
VERSION = "0.04"

# I2C setup for ADXL345
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # ADXL345 on I2C0

# SPI setup for ILI9341
spi = SPI(0, baudrate=40000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3))
cs = Pin(5, Pin.OUT, value=1)
dc = Pin(13, Pin.OUT)
rst = Pin(14, Pin.OUT)
# Initialize ILI9341 (90° clockwise, 320x240)
WIDTH = 320
HEIGHT = 240
display = ILI9341(spi, cs=cs, dc=dc, rst=rst, width=WIDTH, height=HEIGHT, rotation=1)

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))
uart.write(f"ADXL345 Sensor v{VERSION} Started\n".encode())

# On-board LED
led = Pin("LED", Pin.OUT)
led_state = False
led_toggle_counter = 0
LED_TOGGLE_INTERVAL = 500  # 500 loops * 2ms = 1000ms (1 Hz)

# ADXL345 Registers
ADXL345_ADDR = 0x53
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
BW_RATE = 0x2C
DATAX0 = 0x32

# Window definitions
TOP_WINDOW_Y = 0
TOP_WINDOW_HEIGHT = 120
BOTTOM_WINDOW_Y = 120
BOTTOM_WINDOW_HEIGHT = 120
GRAPH_Y_START = BOTTOM_WINDOW_Y + BOTTOM_WINDOW_HEIGHT - 60  # Bottom 25% (60 pixels)

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

# Global data buffer for interrupt
latest_reading = None
magnitudes = []
max_magnitude = 0.0
readings = []  # For web display (retained for future)
kalman_x = KalmanFilter()
kalman_y = KalmanFilter()
kalman_z = KalmanFilter()
rtc = RTC()

# Initialize ADXL345
def adxl345_init():
    try:
        i2c0.writeto_mem(ADXL345_ADDR, POWER_CTL, bytes([0x08]))
        i2c0.writeto_mem(ADXL345_ADDR, DATA_FORMAT, bytes([0x00 | 0x08]))
        i2c0.writeto_mem(ADXL345_ADDR, BW_RATE, bytes([0x0A]))
    except Exception as e:
        display.fill(0)
        display.text("ADXL345 Error", 0, 0, color565(255, 255, 255))
        display.text(str(e), 0, 20, color565(255, 255, 255))
        display.show()
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
    except Exception:
        return 0, 0, 0

# Convert to g
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 256.0
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Graph filtered magnitude
def draw_magnitude_graph(magnitudes):
    # Draw in bottom 25% of bottom window (y=180 to y=239, 60 pixels)
    if len(magnitudes) > 0:
        max_mag = max(magnitudes)
        # Scale normal noise (e.g., <0.1g) to fit in 25% of graph height
        scale_factor = 60 / max(max_mag, 0.1)  # 60 pixels, cap at 0.1g
        for i in range(min(len(magnitudes), WIDTH)):
            height = int(magnitudes[i] * scale_factor)
            height = min(height, 60)  # Cap at 60 pixels
            display.vline(i, HEIGHT - height - 1, height, color565(0, 255, 0))

# Timer interrupt callback
def timer_callback(timer):
    global latest_reading, magnitudes, max_magnitude, readings
    start_time_us = ticks_us()
    start_time = localtime()

    # Read and process ADXL345 data
    raw_x, raw_y, raw_z = read_accel()
    if raw_x == 0 and raw_y == 0 and raw_z == 0:
        return  # Skip on error

    a_x_raw, a_y_raw, a_z_raw = convert_to_g(raw_x, raw_y, raw_z)
    a_x_filt = kalman_x.update(a_x_raw)
    a_y_filt = kalman_y.update(a_y_raw)
    a_z_filt = kalman_z.update(a_z_raw - 1.0)
    magnitude_filt = math.sqrt(a_x_filt**2 + a_y_filt**2 + a_z_filt**2)

    # Update max magnitude
    max_magnitude = max(max_magnitude, magnitude_filt)

    # Store data
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
        'timestamp_str': timestamp_str
    }
    readings.append({
        'timestamp': timestamp_str,
        'magnitude': magnitude_filt
    })
    if len(readings) > 20:
        readings.pop(0)

# Main setup
# Display version for 2 seconds
display.fill(0)
display.text(f"Version {VERSION}", 120, 100, color565(255, 255, 255))
display.show()
sleep_us(2000000)

if not adxl345_init():
    while True:
        sleep_us(1000000)

# Set local RTC time (temporary, no NTP)
rtc.datetime((2025, 6, 17, 22, 4, 0, 0, 0))  # Set to current date/time: June 17, 2025, 04:04 PM MST (22:04 UTC)

# Initialize timer for 500 Hz (2ms period)
timer = Timer()
timer.init(period=2, mode=Timer.PERIODIC, callback=timer_callback)

# Main loop
sample_interval_us = 2000
while True:
    start_time_us = ticks_us()

    # Toggle LED every 1000ms (500 loops)
    led_toggle_counter += 1
    if led_toggle_counter >= LED_TOGGLE_INTERVAL:
        led_state = not led_state
        led.value(1 if led_state else 0)
        led_toggle_counter = 0

    # Process latest reading
    if latest_reading:
        # Clear display
        display.fill(0)
        # Top window: Network status
        display.text("Network Disabled", 0, TOP_WINDOW_Y, color565(255, 255, 255))
        # Bottom window: Raw g and graph
        raw_str = f"X:{latest_reading['a_x_raw']:.2f} Y:{latest_reading['a_y_raw']:.2f} Z:{latest_reading['a_z_raw']:.2f}"
        display.text(raw_str, 0, BOTTOM_WINDOW_Y, color565(255, 255, 255))
        draw_magnitude_graph(magnitudes)
        display.show()

        # UART output
        output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
            latest_reading['timestamp_ms'],
            latest_reading['a_x_raw'], latest_reading['a_y_raw'], latest_reading['a_z_raw'],
            latest_reading['a_x_filt'], latest_reading['a_y_filt'], latest_reading['a_z_filt'],
            latest_reading['magnitude_filt']
        )
        uart.write(output.encode())

    # Maintain approximate 500 Hz loop
    elapsed_us = ticks_us() - start_time_us
    sleep_us(max(0, sample_interval_us - elapsed_us))