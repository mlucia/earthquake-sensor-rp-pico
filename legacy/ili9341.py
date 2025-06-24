# Version 1.0.4
# ILI9341 driver with default font only
from machine import Pin, SPI
from time import sleep_ms
import framebuf

ILI9341_SWRESET = 0x01
ILI9341_SLPOUT = 0x11
ILI9341_DISPON = 0x29
ILI9341_CASET = 0x2A
ILI9341_PASET = 0x2B
ILI9341_RAMWR = 0x2C
ILI9341_MADCTL = 0x36
ILI9341_COLMOD = 0x3A

MADCTL_MY = 0x80
MADCTL_MX = 0x40
MADCTL_MV = 0x20
MADCTL_ML = 0x10
MADCTL_RGB = 0x00
MADCTL_BGR = 0x08
MADCTL_MH = 0x04

class ILI9341:
    def __init__(self, spi, cs, dc, rst, width=240, height=320, rotation=0):
        self.spi = spi
        self.cs = cs
        self.dc = dc
        self.rst = rst
        self.width = width
        self.height = height
        self.rotation = rotation % 4
        self.buffer = bytearray(width * height * 2)
        self.framebuf = framebuf.FrameBuffer(self.buffer, width, height, framebuf.RGB565)
        
        self.reset()
        self.init_display()
        
    def reset(self):
        self.rst(0)
        sleep_ms(10)
        self.rst(1)
        sleep_ms(120)
        
    def write_cmd(self, cmd):
        self.dc(0)
        self.cs(0)
        self.spi.write(bytes([cmd]))
        self.cs(1)
        
    def write_data(self, data):
        self.dc(1)
        self.cs(0)
        self.spi.write(data)
        self.cs(1)
        
    def init_display(self):
        self.write_cmd(ILI9341_SWRESET)
        sleep_ms(150)
        self.write_cmd(ILI9341_SLPOUT)
        sleep_ms(255)
        
        madctl = 0x00
        if self.rotation == 0:
            madctl = MADCTL_MX | MADCTL_BGR
        elif self.rotation == 1:
            madctl = MADCTL_MV | MADCTL_BGR
        elif self.rotation == 2:
            madctl = MADCTL_MY | MADCTL_BGR
        elif self.rotation == 3:
            madctl = MADCTL_MV | MADCTL_MY | MADCTL_BGR
        self.write_cmd(ILI9341_MADCTL)
        self.write_data(bytes([madctl]))
        
        self.write_cmd(ILI9341_COLMOD)
        self.write_data(bytes([0x55]))
        
        self.write_cmd(ILI9341_DISPON)
        sleep_ms(10)
        
    def set_window(self, x0, y0, x1, y1):
        self.write_cmd(ILI9341_CASET)
        self.write_data(bytes([x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF]))
        self.write_cmd(ILI9341_PASET)
        self.write_data(bytes([y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF]))
        self.write_cmd(ILI9341_RAMWR)
        
    def fill(self, color):
        self.framebuf.fill(color)
        
    def text(self, string, x, y, color):
        self.framebuf.text(string, x, y, color)
        
    def vline(self, x, y, h, color):
        self.framebuf.vline(x, y, h, color)
        
    def fill_rect(self, x, y, w, h, color):
        self.framebuf.fill_rect(x, y, w, h, color)
        
    def rect(self, x, y, w, h, color):
        self.framebuf.rect(x, y, w, h, color)
        
    def show(self):
        self.set_window(0, 0, self.width - 1, self.height - 1)
        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)

def color565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)