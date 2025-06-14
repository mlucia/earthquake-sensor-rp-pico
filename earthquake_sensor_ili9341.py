from machine import I2C, Pin, UART, RTC, Timer, SPI
from utime import sleep_us, ticks_us
import math
import network
import socket
from ili9341 import ILI9341, color565
from framebuf import FrameBuffer, RGB565

# Version number
VERSION = "0.01"

# WiFi credentials
SSID = "xxxx"
PASSWORD = "xxxx"

# WiFi status
wlan = network.WLAN(network.STA_IF)
wifi_connected = False
wifi_retry_time = 0
WIFI_RETRY_INTERVAL = 10000000  # 10 seconds in microseconds
ip_address = "WiFi Connecting"

# I2C setups
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # ADXL345 on I2C0
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)  # RTC on I2C1

# SPI setup for ILI9341
spi = SPI(0, baudrate=40000000, polarity=0, phase=0, sck=Pin(10), mosi=Pin(11))
cs = Pin(12, Pin.OUT, value=1)
dc = Pin(13, Pin.OUT)
rst = Pin(14, Pin.OUT)
# Initialize ILI9341
WIDTH = 240
HEIGHT = 320
display = ILI9341(spi, cs=cs, dc=dc, rst=rst, width=WIDTH, height=HEIGHT, rotation=0)

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(f"ADXL345 Sensor v{VERSION} Started\n".encode())

# On-board LED
led = Pin("LED", Pin.OUT)
led_state = False
led_toggle_counter = 0
LED_TOGGLE_INTERVAL = 500  # 500 loops * 2ms = 1000ms (1 Hz)

# Button setup (Pin 6, GP6, pull-up, active low)
button = Pin(6, Pin.IN, Pin.PULL_UP)
display_page = 0  # 0: IP + raw g + graph, 1: filtered g + time + graph
last_button_time = 0
DEBOUNCE_TIME = 200000  # 200ms in microseconds

# ADXL345 Registers
ADXL345_ADDR = 0x53
POWER_CTL = 0x2D
DATA_FORMAT = 0x31
BW_RATE = 0x2C
DATAX0 = 0x32

# RTC constants
RTC_ADDR = 0x68

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
readings = []  # For web display
kalman_x = KalmanFilter()
kalman_y = KalmanFilter()
kalman_z = KalmanFilter()
server_socket = None

# Button interrupt handler
def button_handler(pin):
    global display_page, last_button_time
    current_time = ticks_us()
    if current_time - last_button_time > DEBOUNCE_TIME:
        display_page = (display_page + 1) % 2  # Toggle between 0 and 1
        last_button_time = current_time

# Initialize button interrupt
button.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

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

# Initialize RTC
rtc = RTC()
def rtc_init():
    try:
        rtc.datetime((2025, 6, 14, 11, 54, 0, 0, 0))  # Updated to current date/time
    except Exception as e:
        display.fill(0)
        display.text("RTC Error", 0, 0, color565(255, 255, 255))
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
    # Draw in bottom area (y=40 to y=319, 280 pixels tall)
    if len(magnitudes) > 0:
        max_mag = max(magnitudes)
        for i in range(min(len(magnitudes), WIDTH)):
            height = int((magnitudes[i] / max(max_mag, 0.1)) * (HEIGHT - 40))  # Scale to 280 pixels
            display.vline(i, HEIGHT - height - 1, height, color565(0, 255, 0))

# Timer interrupt callback
def timer_callback(timer):
    global latest_reading, magnitudes, max_magnitude, readings
    start_time_us = ticks_us()
    start_time = rtc.datetime()

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

    timestamp_ms = (start_time[4] * 3600 + start_time[5] * 60 + start_time[6]) * 1000 + start_time[7] // 1000
    timestamp_str = f"{start_time[4]:02d}:{start_time[5]:02d}:{start_time[6]:02d}.{start_time[7] // 100000:1d}"
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

# WiFi connection (non-blocking)
def connect_to_wifi():
    global wifi_connected, wifi_retry_time, ip_address
    if wifi_connected:
        return wlan.ifconfig()[0] if wlan.isconnected() else None
    if ticks_us() < wifi_retry_time:
        return None
    try:
        wlan.active(True)
        if not wlan.isconnected():
            wlan.connect(SSID, PASSWORD)
            wifi_retry_time = ticks_us() + WIFI_RETRY_INTERVAL
            print('Initiating WiFi connection...')
            ip_address = "WiFi Connecting"
            display.fill(0)
            display.text("WiFi Connecting", 0, 0, color565(255, 255, 255))
            display.show()
        else:
            wifi_connected = True
            ip = wlan.ifconfig()[0]
            print('Connected to WiFi, IP:', ip)
            ip_address = ip
            display.fill(0)
            display.text("WiFi Connected", 0, 0, color565(255, 255, 255))
            display.text(ip, 0, 20, color565(255, 255, 255))
            display.show()
            return ip
    except Exception as e:
        print('WiFi connection error:', str(e))
        wifi_retry_time = ticks_us() + WIFI_RETRY_INTERVAL
        ip_address = "WiFi Error"
        display.fill(0)
        display.text("WiFi Error", 0, 0, color565(255, 255, 255))
        display.text(str(e), 0, 20, color565(255, 255, 255))
        display.show()
        return None
    return None

# Start web server
def start_web_server(ip):
    try:
        addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
        s = socket.socket()
        s.bind(addr)
        s.listen(1)
        s.settimeout(0.01)  # Non-blocking
        print('Web server listening on', addr)
        return s
    except Exception as e:
        print('Web server error:', str(e))
        return None

# HTML for scrolling readings
def webpage(readings):
    html = """<!DOCTYPE html>
<html>
<head>
    <title>Pico 2 W Earthquake Sensor</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        h1 { text-align: center; }
        table { width: 100%; border-collapse: collapse; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
        th { background-color: #f2f2f2; }
        .scrollable { max-height: 400px; overflow-y: auto; }
    </style>
</head>
<body>
    <h1>Earthquake Sensor Readings</h1>
    <div class="scrollable">
        <table>
            <tr><th>Timestamp</th><th>Magnitude (g)</th></tr>
"""
    for reading in readings:
        timestamp = reading['timestamp']
        magnitude = reading['magnitude']
        html += f"            <tr><td>{timestamp}</td><td>{magnitude:.2f}</td></tr>\n"
    html += """        </table>
    </div>
</body>
</html>"""
    return html

# Main setup
# Display version for 2 seconds
display.fill(0)
display.text(f"Version {VERSION}", 80, 150, color565(255, 255, 255))
display.show()
sleep_us(2000000)

if not adxl345_init() or not rtc_init():
    while True:
        sleep_us(1000000)

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

    # Check WiFi and start web server if connected
    if not wifi_connected:
        ip = connect_to_wifi()
        if ip and server_socket is None:
            server_socket = start_web_server(ip)

    # Process latest reading
    if latest_reading:
        # OLED display based on page
        display.fill(0)
        if display_page == 0:
            # Page 1: IP + raw g + graph
            display.text(ip_address, 0, 0, color565(255, 255, 255))
            raw_str = f"X:{latest_reading['a_x_raw']:.2f} Y:{latest_reading['a_y_raw']:.2f} Z:{latest_reading['a_z_raw']:.2f}"
            display.text(raw_str, 0, 20, color565(255, 255, 255))
            draw_magnitude_graph(magnitudes)
        else:
            # Page 2: filtered g + time + graph
            filt_str = f"X:{latest_reading['a_x_filt']:.2f} Y:{latest_reading['a_y_filt']:.2f} Z:{latest_reading['a_z_filt']:.2f}"
            display.text(filt_str, 0, 0, color565(255, 255, 255))
            time_str = f"Time: {latest_reading['timestamp_str'].split('.')[0]}"
            display.text(time_str, 0, 20, color565(255, 255, 255))
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

    # Handle web server requests only if connected
    if wifi_connected and server_socket:
        try:
            cl, addr = server_socket.accept()
            request = cl.recv(1024)
            request = str(request)
            if 'GET / ' in request or 'GET /index.html' in request:
                response = webpage(readings)
                cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
                cl.send(response)
            cl.close()
        except OSError:
            pass

    # Maintain approximate 500 Hz loop
    elapsed_us = ticks_us() - start_time_us
    sleep_us(max(0, sample_interval_us - elapsed_us))
