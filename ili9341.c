#include "ili9341.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

// 8x8 font (subset, printable ASCII)
static const uint8_t font_8x8[95][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x00}, // !
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // "
    {0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00}, // #
    {0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00}, // $
    // Add more characters as needed
    {0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00}, // :
    // Full font table omitted for brevity
};

static uint cs_pin, dc_pin, rst_pin;
static uint16_t buffer[320 * 240];

void ili9341_init(uint cs, uint dc, uint rst) {
    cs_pin = cs;
    dc_pin = dc;
    rst_pin = rst;
    gpio_init(cs_pin);
    gpio_init(dc_pin);
    gpio_init(rst_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_set_dir(dc_pin, GPIO_OUT);
    gpio_set_dir(rst_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
    
    ili9341_reset();
    ili9341_write_cmd(ILI9341_SWRESET);
    sleep_ms(150);
    ili9341_write_cmd(ILI9341_SLPOUT);
    sleep_ms(255);
    
    ili9341_write_cmd(ILI9341_MADCTL);
    uint8_t madctl = MADCTL_MV | MADCTL_BGR;
    ili9341_write_data(&madctl, 1);
    
    ili9341_write_cmd(ILI9341_COLMOD);
    uint8_t colmod = 0x55;
    ili9341_write_data(&colmod, 1);
    
    ili9341_write_cmd(ILI9341_DISPON);
    sleep_ms(10);
}

void ili9341_reset() {
    gpio_put(rst_pin, 0);
    sleep_ms(10);
    gpio_put(rst_pin, 1);
    sleep_ms(120);
}

void ili9341_write_cmd(uint8_t cmd) {
    gpio_put(dc_pin, 0);
    gpio_put(cs_pin, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    gpio_put(cs_pin, 1);
}

void ili9341_write_data(uint8_t *data, size_t len) {
    gpio_put(dc_pin, 1);
    gpio_put(cs_pin, 0);
    spi_write_blocking(SPI_PORT, data, len);
    gpio_put(cs_pin, 1);
}

void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    ili9341_write_cmd(ILI9341_CASET);
    uint8_t data[4] = {x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF};
    ili9341_write_data(data, 4);
    ili9341_write_cmd(ILI9341_PASET);
    data[0] = y0 >> 8; data[1] = y0 & 0xFF;
    data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    ili9341_write_data(data, 4);
    ili9341_write_cmd(ILI9341_RAMWR);
}

void ili9341_fill(uint16_t color) {
    for (int i = 0; i < 320 * 240; i++) buffer[i] = color;
}

void ili9341_text(const char *str, uint16_t x, uint16_t y, uint16_t color) {
    while (*str) {
        if (*str >= 32 && *str <= 126) {
            const uint8_t *glyph = font_8x8[*str - 32];
            for (int row = 0; row < 8; row++) {
                for (int col = 0; col < 8; col++) {
                    if (glyph[row] & (1 << (7 - col))) {
                        if (x + col < 320 && y + row < 240) {
                            buffer[(y + row) * 320 + (x + col)] = color;
                        }
                    }
                }
            }
        }
        x += 8;
        str++;
    }
}

void ili9341_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
    if (x >= 320 || y >= 240) return;
    if (y + h > 240) h = 240 - y;
    for (int i = y; i < y + h; i++) {
        buffer[i * 320 + x] = color;
    }
}

void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= 320 || y >= 240) return;
    if (x + w > 320) w = 320 - x;
    if (y + h > 240) h = 240 - y;
    for (int i = y; i < y + h; i++) {
        for (int j = x; j < x + w; j++) {
            buffer[i * 320 + j] = color;
        }
    }
}

void ili9341_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    ili9341_vline(x, y, h, color);
    ili9341_vline(x + w - 1, y, h, color);
    for (int i = y; i < y + h; i++) {
        buffer[i * 320 + x] = color;
        buffer[i * 320 + (x + w - 1)] = color;
    }
}

void ili9341_show() {
    ili9341_set_window(0, 0, 319, 239);
    gpio_put(dc_pin, 1);
    gpio_put(cs_pin, 0);
    spi_write_blocking(SPI_PORT, (uint8_t *)buffer, 320 * 240 * 2);
    gpio_put(cs_pin, 1);
}