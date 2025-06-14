from machine import I2C, Pin, UART, RTC, Timer
from utime import sleep_us, ticks_us
import math
import network
import socket
from ssd1306 import SSD1306_I2C

# WiFi credentials
SSID = "Lucia1"
PASSWORD = "scoobydooby"

# I2C setups
i2c0 = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # ADXL345 on I2C0
i2c1 = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)  # RTC and OLED on I2C1

# UART setup
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.write(b"ADXL345 Earthquake Sensor with Timer Interrupt Started\n")

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

# Global data buffer for interrupt
latest_reading = None
magnitudes = []
last_event_time = None
max_magnitude = 0.0
graph_display_start = 0
readings = []  # For web display
kalman_x = KalmanFilter()
kalman_y = KalmanFilter()
kalman_z = KalmanFilter()
threshold_g = 0.1

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
        rtc.datetime((2025, 6, 14, 10, 41, 0, 0, 0))  # Updated to current date
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
    except Exception:
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

# Timer interrupt callback
def timer_callback(timer):
    global latest_reading, magnitudes, last_event_time, max_magnitude, graph_display_start, readings
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
    magnitude_raw = math.sqrt(a_x_raw**2 + a_y_raw**2 + a_z_raw**2)
    magnitude_filt = math.sqrt(a_x_filt**2 + a_y_filt**2 + a_z_filt**2)
    seismic_event = magnitude_raw > threshold_g or magnitude_filt > threshold_g

    # Update event tracking
    if seismic_event:
        last_event_time = start_time
        max_magnitude = max(max_magnitude, magnitude_filt)
        graph_display_start = ticks_us()

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
        'seismic_event': seismic_event,
        'magnitude_filt': magnitude_filt,
        'timestamp_str': timestamp_str
    }
    readings.append({
        'timestamp': timestamp_str,
        'magnitude': magnitude_filt,
        'event': seismic_event
    })
    if len(readings) > 20:
        readings.pop(0)

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
        ip = wlan.ifconfig()[0]
        print('Connected to WiFi, IP:', ip)
        oled.fill(0)
        oled.text("WiFi Connected", 0, 0)
        oled.text("IP: " + ip, 0, 10)
        oled.show()
        sleep_us(2000000)
        return ip

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
            <tr><th>Timestamp</th><th>Magnitude (g)</th><th>Event</th></tr>
"""
    for reading in readings:
        timestamp = reading['timestamp']
        magnitude = reading['magnitude']
        event = reading['event']
        html += f"            <tr><td>{timestamp}</td><td>{magnitude:.2f}</td><td>{'Yes' if event else 'No'}</td></tr>\n"
    html += """        </table>
    </div>
</body>
</html>"""
    return html

# Start web server
def start_web_server(ip):
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.bind(addr)
    s.listen(1)
    s.settimeout(0.01)  # Non-blocking
    print('Web server listening on', addr)
    return s

# Main setup
if not adxl345_init() or not rtc_init():
    while True:
        sleep_us(1000000)

# Connect to WiFi
ip = connect_to_wifi()

# Start web server
server_socket = start_web_server(ip)

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
        # OLED display logic
        if latest_reading['seismic_event'] and (ticks_us() - graph_display_start) < 2000000:
            draw_seismic_graph(magnitudes)
        else:
            oled.fill(0)
            oled.text("Status: Running", 0, 0)
            oled.text(f"Time: {latest_reading['timestamp_str']}", 0, 10)
            if last_event_time:
                event_str = f"Last: {last_event_time[4]:02d}:{last_event_time[5]:02d}:{last_event_time[6]:02d}"
                oled.text(event_str, 0, 20)
            else:
                oled.text("Last: None", 0, 20)
            oled.text(f"Max Mag: {max_magnitude:.2f}g", 0, 30)
            oled.show()

        # UART output
        output = "{:d},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:d}\n".format(
            latest_reading['timestamp_ms'],
            latest_reading['a_x_raw'], latest_reading['a_y_raw'], latest_reading['a_z_raw'],
            latest_reading['a_x_filt'], latest_reading['a_y_filt'], latest_reading['a_z_filt'],
            1 if latest_reading['seismic_event'] else 0
        )
        uart.write(output.encode())

    # Handle web server requests
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