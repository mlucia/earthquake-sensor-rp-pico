# Earthquake Sensor for Raspberry Pi Pico 2 W

This project implements a seismic sensor using a Raspberry Pi Pico 2 W, an ADXL345 accelerometer, an ADS1115 ADC, and an ILI9341 2.8" TFT display. Version `1.3.0` fixes a `SyntaxError` in `main.py`, optimizes ADS1115 reading, miniSEED processing, graph drawing, and timer scheduling to resolve display update delays, with SPI baudrate at 30 MHz.

## Features
- **Version**: `1.3.0` (displayed for 2s at boot).
- **Display**: 90° rotated (320x240), split into top (y=0–119) and bottom (y=120–239) windows with white borders.
  - Top: "Network Disabled" (y=8, 8x8 font).
  - Bottom: Raw g values (y=128, 8x8 font), A0 voltage (y=144, yellow), graph (y=180–239, bottom 25%).
- **Sampling**: 100 Hz for ADXL345 and ADS1115 with Kalman filtering.
- **Time**: Local RTC (June 20, 2025, 01:41 PM MST).
- **UART**: Outputs miniSEED records (32-byte header + 6-byte data) at 100 Hz (TX=Pin 8, 115200 baud), plus frame time diagnostics.
- **ADS1115**: Reads A0 voltage (6.144V range), displayed in bottom window.
- **Test Program**: `test_ili9341.py` tests text, lines, color bars, and "TEST" with default font.

## Libraries
- `ili9341.py`: v1.0.4, ILI9341 driver, default 8x8 font, based on [rdagger/micropython-ili9341](https://github.com/rdagger/micropython-ili9341), MIT License.

## Hardware Setup
- **Pico 2 W**: MicroPython v1.23+.
- **ILI9341**: SPI, SCK=Pin 2, MOSI=Pin 3, CS=Pin 5, DC=Pin 13, RESET=Pin 14, VCC=3.3V/5V, GND=Pin 38, backlight=Pin 15 or 3.3V.
- **ADXL345**: I2C0, SDA=Pin 0, SCL=Pin 1, VCC=3.3V, GND=Pin 38.
- **ADS1115**: I2C0, SDA=Pin 0, SCL=Pin 1, VCC=3.3V, GND=Pin 38, ADDR=GND (0x48).
  - A0: Connect to voltage source (0–6.144V) or leave unconnected for testing.
- **UART**: TX=Pin 8, RX=Pin 9.

## Software Setup
1. Install MicroPython v1.23+ on Pico.
2. Connect Pico to Thonny, select “MicroPython (Raspberry Pi Pico)” in “Tools > Options > Backend”.
3. Delete existing files on Pico:
   - In Thonny’s file browser (View > Files), right-click and delete `main.py`, `ili9341.py`, `test_ili9341.py`.
   - Verify: `import os; print(os.listdir())` (expect `[]` or unrelated files).
4. Upload files (`main.py`, `ili9341.py`, `test_ili9341.py`):
   - Right-click each file in local directory, select “Upload to /”.
   - Or use Thonny shell:
     ```python
     import os
     for file in ['main.py', 'ili9341.py', 'test_ili9341.py']:
         with open(file, 'w') as f:
             f.write(open(f'/local/path/to/{file}').read())
     ```
5. Verify: `import os; print(os.listdir())` (expect `['main.py', 'ili9341.py', 'test_ili9341.py']`).

## Running
1. **Main**: Displays "Version 1.3.0" (2s), "Network Disabled" (top), raw g, A0 voltage, graph (bottom, ~3.2s to fill), miniSEED via UART.
2. **Test**: Run `test_ili9341.py` for text, lines, bars, and "TEST".

## Testing
- **Display**: Verify 8x8 font text, A0 voltage, borders, graph fills in ~3.2s.
- **I2C**: `i2c0.scan()` (expect `0x53`, `0x48`).
- **UART**: Check TX=Pin 8, 115200 baud, miniSEED (38 bytes/record), frame time (~10–15 ms).
- **ADS1115**: Connect A0 to 3.3V, verify display (~3.3V).
- **Time**: 2025-06-20 01:41:00 MST.

## Troubleshooting
- **SyntaxError**:
  - Verify `main.py` version (`# Version 1.3.0`).
  - Re-upload files to ensure no corruption.
- **Display Delays**:
  - Check UART for frame time (should be <15 ms).
  - Reduce SPI baudrate to `20000000` in `main.py` and `test_ili9341.py`.
  - Test without ADS1115: Comment out `latest_reading['voltage'] = read_ads1115()`.
  - Disable miniSEED: Comment out `uart.write(miniseed)`.
- **I2C**: Check SDA/SCL (Pins 0–1), pull-ups (4.7kΩ to 3.3V if needed).
- **UART**: Use serial monitor to verify miniSEED and frame time.
- **ADS1115**: If A0 voltage is 0V, check wiring or test with `read_ads1115()` in Thonny shell.
- **Thonny Errors**: Restart Thonny, re-upload files, update to latest version.

## Future Improvements
- Add multi-channel miniSEED support.
- Re-enable WiFi/NTP/OTA.
- Add SD card logging.
- Enable touchscreen (XPT2046).

## License
MIT License (see `LICENSE` if added).