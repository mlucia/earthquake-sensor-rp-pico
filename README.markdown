# Earthquake Sensor for Raspberry Pi Pico 2 W

This project implements a seismic sensor using a Raspberry Pi Pico 2 W, an ADXL345 accelerometer, an ADS1115 ADC, and an ILI9341 2.8" TFT display, written in C/C++ with the Pico SDK. Version `1.1.1` migrates from MicroPython (v1.1.0), using default 8x8 font, 20 MHz SPI, miniSEED UART output, A0 voltage display, and 100 Hz sampling.

## Features
- **Version**: `1.1.1` (displayed for 2s at boot).
- **Display**: 90° rotated (320x240), split into top (y=0–119) and bottom (y=120–239) windows with white borders.
  - Top: "Network Disabled" (y=8, 8x8 font).
  - Bottom: Raw g values (y=128, 8x8 font), A0 voltage (y=144, yellow), graph (y=180–239, bottom 25%).
- **Sampling**: 100 Hz for ADXL345 and ADS1115 with Kalman filtering.
- **Time**: RTC set to 2025-06-24 01:06 PM MST (20:06 UTC).
- **UART**: miniSEED records at 100 Hz (TX=Pin 8, 115200 baud, 48-byte header + 6-byte data).
- **ADS1115**: Reads A0 voltage (6.144V range), displayed in bottom window.
- **Test Program**: `test_ili9341.c` tests text, lines, color bars, and "TEST" with 8x8 font.

## Hardware Setup
- **Pico 2 W**: Pico SDK v2.0.0+.
- **ILI9341**: SPI0, SCK=GP2, MOSI=GP3, CS=GP5, DC=GP13, RESET=GP14, VCC=3.3V/5V, GND=Pin 38, backlight=Pin 15 or 3.3V.
- **ADXL345**: I2C0, SDA=GP0, SCL=GP1, VCC=3.3V, GND=Pin 38.
- **ADS1115**: I2C0, SDA=GP0, SCL=GP1, VCC=3.3V, GND=Pin 38, ADDR=GND (0x48).
  - A0: Connect to voltage source (0–6.144V) or leave unconnected for testing.
- **UART**: TX=GP8, RX=GP9.

## Software Setup
1. Install Pico SDK (v2.0.0+): Follow [Raspberry Pi Pico C/C++ SDK](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).
2. Set up build environment (e.g., Pico SDK, CMake, GCC for ARM).
3. Clone repository: `git clone https://github.com/mlucia/earthquake-sensor-rp-pico.git`.
4. Copy `main.c`, `ili9341.c`, `ili9341.h`, `test_ili9341.c`, `CMakeLists.txt` to project directory.
5. Build and flash:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```
   Flash `main.uf2` or `test_ili9341.uf2` to Pico via USB (BOOTSEL mode).
6. Verify files: `main.c`, `ili9341.c`, `ili9341.h`, `test_ili9341.c`, `CMakeLists.txt`.

## Running
1. **Main**: Flash `main.uf2`. Displays "Version 1.1.1" (2s), "Network Disabled" (top), raw g, A0 voltage, graph (bottom), miniSEED via UART.
2. **Test**: Flash `test_ili9341.uf2`. Tests text, lines, bars, and "TEST".

## Testing
- **Display**: Verify readable 8x8 font, A0 voltage, borders, graph.
- **I2C**: Use debug tool or modify `main.c` to print `i2c_read_blocking` results (expect `0x53`, `0x48`).
- **UART**: Monitor TX=GP8, 115200 baud, for 54-byte miniSEED records.
- **ADS1115**: Connect A0 to 3.3V, verify ~3.3V on display.
- **Time**: 2025-06-24 01:06:00 MST.

## Troubleshooting
- **Display Issues**:
  - Check SPI wiring (SCK=GP2, MOSI=GP3, CS=GP5, DC=GP13, RESET=GP14).
  - Reduce SPI baudrate to `10000000` in `main.c` and `test_ili9341.c`.
- **I2C**: Verify SDA/SCL (GP0, GP1), add 4.7kΩ pull-ups to 3.3V if needed.
- **UART**: Use serial monitor to verify miniSEED output.
- **ADS1115**: If A0 voltage is 0V, check A0 wiring or debug `read_ads1115()`.
- **Build Errors**: Ensure Pico SDK is correctly installed and paths are set.

## Future Improvements
- Add multi-channel miniSEED support.
- Enable WiFi/NTP/OTA (requires RP2040 WiFi libraries).
- Add SD card logging.
- Enable touchscreen (XPT2046).

## License
MIT License (see `LICENSE` if added).