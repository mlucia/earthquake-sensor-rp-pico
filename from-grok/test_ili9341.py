# Version 1.0.4
from machine import Pin, SPI
from utime import sleep_us
from ili9341 import ILI9341, color565
from framebuf import FrameBuffer, RGB565

spi = SPI(0, baudrate=20000000, polarity=0, phase=0, sck=Pin(2), mosi=Pin(3))
cs = Pin(5, Pin.OUT, value=1)
dc = Pin(13, Pin.OUT)
rst = Pin(14, Pin.OUT)
WIDTH = 320
HEIGHT = 240
display = ILI9341(spi, cs=cs, dc=dc, rst=rst, width=WIDTH, height=HEIGHT, rotation=1)

def test_display():
    display.fill(color565(0, 0, 0))
    display.show()
    sleep_us(1000000)

    # Test 1: Colored background and text (default font)
    display.fill(color565(0, 0, 255))
    display.text("ILI9341 Test", 20, 20, color565(255, 255, 255))
    display.text("320x240 TFT", 20, 36, color565(255, 255, 0))
    display.text("Pins: 2,3,5,13,14", 20, 52, color565(255, 0, 0))
    display.show()
    sleep_us(2000000)

    # Test 2: Vertical lines (red)
    display.fill(0)
    for x in range(0, WIDTH, 10):
        display.vline(x, 0, HEIGHT, color565(255, 0, 0))
    display.text("Red Lines", 20, 20, color565(255, 255, 255))
    display.show()
    sleep_us(2000000)

    # Test 3: Color bars
    display.fill(0)
    bar_width = WIDTH // 4
    display.fill_rect(0, 0, bar_width, HEIGHT, color565(255, 0, 0))
    display.fill_rect(bar_width, 0, bar_width, HEIGHT, color565(0, 255, 0))
    display.fill_rect(2 * bar_width, 0, bar_width, HEIGHT, color565(0, 0, 255))
    display.fill_rect(3 * bar_width, 0, bar_width, HEIGHT, color565(255, 255, 255))
    display.text("Color Bars", 20, 20, color565(0, 0, 0))
    display.show()
    sleep_us(2000000)

    # Test 4: Default font text
    display.fill(0)
    display.text("Default Font", 20, 20, color565(255, 255, 255))
    display.text("8x8 Font", 20, 36, color565(255, 255, 0))
    display.text("Version 1.0.4", 20, 52, color565(255, 0, 0))
    display.show()
    sleep_us(2000000)

    # Test 5: Single word large font (default)
    display.fill(0)
    display.text("TEST", 96, 96, color565(255, 255, 255))
    display.show()
    sleep_us(2000000)

try:
    test_display()
except Exception as e:
    display.fill(0)
    display.text("Error", 0, 0, color565(255, 255, 255))
    display.text(str(e), 0, 16, color565(255, 255, 255))
    display.show()