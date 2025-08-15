//#include <pico/stdio.h>
//#include <string.h>
#include <math.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
//#include "hardware/rtc.h"
#include "ili9341.h"
//#include "pico/util/datetime.h"

#define VERSION "1.1.1"

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C_FREQ 400000

#define SPI_PORT spi0
#define SPI_SCK 2
#define SPI_MOSI 3
#define SPI_CS 5
#define SPI_DC 13
#define SPI_RST 14
#define WIDTH 320
#define HEIGHT 240

#define UART_ID uart1
#define UART_TX 8
#define UART_RX 9
#define UART_BAUD 115200

#define ADXL345_ADDR 0x53
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define BW_RATE 0x2C
#define DATAX0 0x32
#define ADS1115_ADDR 0x48

#define TOP_WINDOW_Y 0
#define TOP_WINDOW_HEIGHT 120
#define BOTTOM_WINDOW_Y 120
#define BOTTOM_WINDOW_HEIGHT 120
#define GRAPH_Y_START (BOTTOM_WINDOW_Y + BOTTOM_WINDOW_HEIGHT - 60)

typedef struct {
    float x, p, q, r, k;
} KalmanFilter;

typedef struct {
    uint32_t timestamp_ms;
    float a_x_raw, a_y_raw, a_z_raw;
    float a_x_filt, a_y_filt, a_z_filt;
    float magnitude_filt;
    char timestamp_str[9];
    float voltage;
} Reading;

void kalman_init(KalmanFilter *kf) {
    kf->x = 0.0f;
    kf->p = 1.0f;
    kf->q = 0.001f;
    kf->r = 0.00015f;
    kf->k = 0.0f;
}

float kalman_update(KalmanFilter *kf, float measurement) {
    kf->p = kf->p + kf->q;
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (0.5f - kf->k) * kf->p;
    return kf->x;
}

Reading latest_reading = {0};
float magnitudes[WIDTH] = {0};
float max_magnitude = 0.0f;
int magnitude_count = 0;
KalmanFilter kalman_x, kalman_y, kalman_z;

bool adxl345_init() {
    uint8_t data[2];
    data[0] = POWER_CTL;
    data[1] = 0x08; // Measurement mode
    if (i2c_write_blocking(I2C_PORT, ADXL345_ADDR, data, 2, false) < 0) goto error;
    data[0] = DATA_FORMAT;
    data[1] = 0x08; // Full resolution
    if (i2c_write_blocking(I2C_PORT, ADXL345_ADDR, data, 2, false) < 0) goto error;
    data[0] = BW_RATE;
    data[1] = 0x09; // 100 Hz
    if (i2c_write_blocking(I2C_PORT, ADXL345_ADDR, data, 2, false) < 0) goto error;
    return true;
error:
    ili9341_fill(0);
    ili9341_text("ADXL345 Error", 0, 0, ILI9341_COLOR(255, 255, 255));
    ili9341_show();
    return false;
}

bool ads1115_init() {
    uint8_t addr_list[2];
    int num = i2c_read_blocking(I2C_PORT, 0x00, addr_list, 2, false);
    for (int i = 0; i < num; i++) {
        if (addr_list[i] == ADS1115_ADDR) return true;
    }
    ili9341_fill(0);
    ili9341_text("ADS1115 Not Found", 0, 0, ILI9341_COLOR(255, 255, 255));
    ili9341_show();
    return false;
}

float read_ads1115() {
    uint8_t config[3] = {0x01, 0xC1, 0xE3}; // A0, 6.144V, 128 sps
    if (i2c_write_blocking(I2C_PORT, ADS1115_ADDR, config, 3, false) < 0) return 0.0f;
    sleep_us(10000);
    uint8_t data[2];
    if (i2c_read_blocking(I2C_PORT, ADS1115_ADDR, data, 2, false) < 2) return 0.0f;
    int16_t raw = (data[0] << 8) | data[1];
    return (raw / 32768.0f) * 6.144f;
}

bool read_accel(int16_t *a_x, int16_t *a_y, int16_t *a_z) {
    uint8_t reg = DATAX0;
    uint8_t data[6];
    if (i2c_write_blocking(I2C_PORT, ADXL345_ADDR, &reg, 1, true) < 0) return false;
    if (i2c_read_blocking(I2C_PORT, ADXL345_ADDR, data, 6, false) < 6) return false;
    *a_x = (data[1] << 8) | data[0];
    *a_y = (data[3] << 8) | data[2];
    *a_z = (data[5] << 8) | data[4];
    return true;
}

void convert_to_g(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *a_x, float *a_y, float *a_z) {
    float accel_scale = 256.0f;
    *a_x = raw_x / accel_scale;
    *a_y = raw_y / accel_scale;
    *a_z = raw_z / accel_scale;
}

void draw_magnitude_graph() {
    if (magnitude_count == 0) return;
    float max_mag = max_magnitude;
    float scale_factor = 60.0f / (max_mag > 0.1f ? max_mag : 0.1f);
    for (int i = 0; i < magnitude_count && i < WIDTH; i++) {
        int height = (int)(magnitudes[i] * scale_factor);
        if (height > 60) height = 60;
        ili9341_vline(i, HEIGHT - height - 1, height, ILI9341_COLOR(0, 255, 0));
    }
}

void draw_borders() {
    ili9341_rect(0, TOP_WINDOW_Y, WIDTH, TOP_WINDOW_HEIGHT, ILI9341_COLOR(255, 255, 255));
    ili9341_rect(0, BOTTOM_WINDOW_Y, WIDTH, BOTTOM_WINDOW_HEIGHT, ILI9341_COLOR(255, 255, 255));
}

void create_miniseed_record(uint32_t timestamp_ms, float a_x, float a_y, float a_z, uint8_t *buffer) {
    memset(buffer, 0, 54);
    sprintf((char *)buffer, "%06lu", timestamp_ms % 1000000);
    buffer[6] = 'D';
    memcpy(buffer + 8, "PICO", 4);
    memcpy(buffer + 12, "LH", 2);
    memcpy(buffer + 14, "Z ", 2);
    memcpy(buffer + 16, "XX", 2);
    datetime_t dt;
    rtc_get_datetime(&dt);
    uint16_t year = dt.year;
    uint16_t day = (dt.month << 8) | dt.day;
    buffer[20] = year >> 8; buffer[21] = year & 0xFF;
    buffer[22] = day >> 8; buffer[23] = day & 0xFF;
    buffer[24] = dt.hour;
    buffer[25] = dt.min;
    buffer[26] = dt.sec;
    buffer[28] = 0; buffer[29] = 0; // Microseconds
    buffer[30] = 0; buffer[31] = 100; // Sample rate
    buffer[32] = 0; buffer[33] = 3; // Number of samples
    int16_t x = (int16_t)(a_x * 1000);
    int16_t y = (int16_t)(a_y * 1000);
    int16_t z = (int16_t)(a_z * 1000);
    buffer[48] = x >> 8; buffer[49] = x & 0xFF;
    buffer[50] = y >> 8; buffer[51] = y & 0xFF;
    buffer[52] = z >> 8; buffer[53] = z & 0xFF;
}

bool timer_callback(repeating_timer_t *rt) {
    uint64_t start_time_us = time_us_64();
    datetime_t dt;
    rtc_get_datetime(&dt);
    
    int16_t raw_x, raw_y, raw_z;
    if (!read_accel(&raw_x, &raw_y, &raw_z)) return true;

    float a_x_raw, a_y_raw, a_z_raw;
    convert_to_g(raw_x, raw_y, raw_z, &a_x_raw, &a_y_raw, &a_z_raw);
    float a_x_filt = kalman_update(&kalman_x, a_x_raw);
    float a_y_filt = kalman_update(&kalman_y, a_y_raw);
    float a_z_filt = kalman_update(&kalman_z, a_z_raw - 1.0f);
    float magnitude_filt = sqrtf(a_x_filt * a_x_filt + a_y_filt * a_y_filt + a_z_filt * a_z_filt);
    float voltage = read_ads1115();

    if (magnitude_filt > max_magnitude) max_magnitude = magnitude_filt;
    if (magnitude_count < WIDTH) {
        magnitudes[magnitude_count++] = magnitude_filt;
    } else {
        memmove(magnitudes, magnitudes + 1, (WIDTH - 1) * sizeof(float));
        magnitudes[WIDTH - 1] = magnitude_filt;
    }

    uint32_t timestamp_ms = (dt.hour * 3600 + dt.min * 60 + dt.sec) * 1000;
    char timestamp_str[9];
    snprintf(timestamp_str, 9, "%02d:%02d:%02d", dt.hour, dt.min, dt.sec);
    latest_reading.timestamp_ms = timestamp_ms;
    latest_reading.a_x_raw = a_x_raw;
    latest_reading.a_y_raw = a_y_raw;
    latest_reading.a_z_raw = a_z_raw;
    latest_reading.a_x_filt = a_x_filt;
    latest_reading.a_y_filt = a_y_filt;
    latest_reading.a_z_filt = a_z_filt;
    latest_reading.magnitude_filt = magnitude_filt;
    strncpy(latest_reading.timestamp_str, timestamp_str, 9);
    latest_reading.voltage = voltage;

    uint8_t miniseed[54];
    create_miniseed_record(timestamp_ms, a_x_filt, a_y_filt, a_z_filt, miniseed);
    uart_write_blocking(UART_ID, miniseed, 54);

    return true;
}

int main() {
    stdio_init_all();
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    spi_init(SPI_PORT, 20000000);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    ili9341_init(SPI_CS, SPI_DC, SPI_RST);

    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_RX, GPIO_FUNC_UART);
    char start_msg[32];
    snprintf(start_msg, 32, "ADXL345 Sensor v%s Started\n", VERSION);
    uart_puts(UART_ID, start_msg);

    kalman_init(&kalman_x);
    kalman_init(&kalman_y);
    kalman_init(&kalman_z);

    ili9341_fill(0);
    char version_str[16];
    snprintf(version_str, 16, "Version %s", VERSION);
    ili9341_text(version_str, 80, 100, ILI9341_COLOR(255, 255, 255));
    ili9341_show();
    sleep_ms(2000);

    if (!adxl345_init() || !ads1115_init()) {
        while (true) sleep_ms(1000);
    }

    datetime_t dt = {
        .year = 2025, .month = 6, .day = 24,
        .hour = 20, .min = 6, .sec = 0
    };
    rtc_init();
    rtc_set_datetime(&dt);

    struct repeating_timer timer;
    add_repeating_timer_ms(10, timer_callback, NULL, &timer);

    while (true) {
        uint64_t start_time_us = time_us_64();
        if (latest_reading.timestamp_ms) {
            ili9341_fill(0);
            ili9341_text("Network Disabled", 8, TOP_WINDOW_Y + 8, ILI9341_COLOR(255, 255, 255));
            char raw_str[32];
            snprintf(raw_str, 32, "X:%.2f Y:%.2f Z:%.2f", 
                     latest_reading.a_x_raw, latest_reading.a_y_raw, latest_reading.a_z_raw);
            ili9341_text(raw_str, 8, BOTTOM_WINDOW_Y + 8, ILI9341_COLOR(255, 255, 255));
            char volt_str[16];
            snprintf(volt_str, 16, "A0: %.3fV", latest_reading.voltage);
            ili9341_text(volt_str, 8, BOTTOM_WINDOW_Y + 24, ILI9341_COLOR(255, 255, 0));
            draw_magnitude_graph();
            draw_borders();
            ili9341_show();
        }
        uint64_t elapsed_us = time_us_64() - start_time_us;
        if (elapsed_us < 10000) sleep_us(10000 - elapsed_us);
    }
    return 0;
}