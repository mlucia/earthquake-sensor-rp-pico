from machine import I2C, Pin
from utime import sleep

# I2C setup (I2C0: SDA=Pin 0, SCL=Pin 1)
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)

# ADXL345 constants
ADXL345_ADDR = 0x53  # I2C address (ALT ADDRESS pin grounded)
POWER_CTL = 0x2D     # Power control register
DATA_FORMAT = 0x31   # Data format register
DATAX0 = 0x32        # X-axis data start register

# Initialize ADXL345
def adxl345_init():
    try:
        # Set to measurement mode
        i2c.writeto_mem(ADXL345_ADDR, POWER_CTL, bytes([0x08]))
        # Full resolution, ±2g range
        i2c.writeto_mem(ADXL345_ADDR, DATA_FORMAT, bytes([0x00 | 0x08]))
        print("ADXL345 initialized successfully")
        return True
    except Exception as e:
        print("ADXL345 initialization failed:", str(e))
        return False

# Read acceleration data
def read_accel():
    try:
        # Read 6 bytes (X0, X1, Y0, Y1, Z0, Z1)
        data = i2c.readfrom_mem(ADXL345_ADDR, DATAX0, 6)
        # Convert to signed 16-bit integers
        a_x = (data[1] << 8 | data[0]) if (data[1] & 0x80) == 0 else (data[1] << 8 | data[0]) - 0x10000
        a_y = (data[3] << 8 | data[2]) if (data[3] & 0x80) == 0 else (data[3] << 8 | data[2]) - 0x10000
        a_z = (data[5] << 8 | data[4]) if (data[5] & 0x80) == 0 else (data[5] << 8 | data[4]) - 0x10000
        return a_x, a_y, a_z
    except Exception as e:
        print("Error reading ADXL345:", str(e))
        return None

# Convert raw data to g units
def convert_to_g(raw_x, raw_y, raw_z):
    accel_scale = 256.0  # For ±2g range in full resolution
    return raw_x / accel_scale, raw_y / accel_scale, raw_z / accel_scale

# Main test loop
if not adxl345_init():
    print("Stopping due to initialization failure")
else:
    print("Reading ADXL345 data (Ctrl+C to stop)...")
    while True:
        accel_data = read_accel()
        if accel_data:
            x, y, z = convert_to_g(*accel_data)
            print(x,  y, z)
        else:
            print("Failed to read data")
        sleep(0.5)  # Read every 500ms