#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ili9341.h"

#define SPI_PORT spi0
#define SPI_SCK 2
#define SPI_MOSI 3
#define SPI_CS 5
#define SPI_DC 13
#define SPI_RST 14
#define WIDTH 320
#define HEIGHT 240

void test_display() {
    ili9341_fill(ILI9341_COLOR(0, 0, 0));
    ili9341_show();
    sleep_ms(1000);

    // Test 1: Colored background and text
    ili9341_fill(ILI9341_COLOR(0, 0, 255));
    ili9341_text("ILI9341 Test", 20, 20, ILI9341_COLOR(255, 255, 255));
    ili9341_text("320x240 TFT", 20, 36, ILI9341_COLOR(255, 255, 0));
    ili9341_text("Pins: 2,3,5,13,14", 20, 52, ILI9341_COLOR(255, 0, 0));
    ili9341_show();
    sleep_ms(2000);

    // Test 2: Vertical lines
    ili9341_fill(0);
    for (int x = 0; x < WIDTH; x += 10) {
        ili9341_vline(x, 0, HEIGHT, ILI9341_COLOR(255, 0, 0));
    }
    ili9341_text("Red Lines", 20, 20, ILI9341_COLOR(255, 255, 255));
    ili9341_show();
    sleep_ms(2000);

    // Test 3: Color bars
    ili9341_fill(0);
    int bar_width = WIDTH / 4;
    ili9341_fill_rect(0, 0, bar_width, HEIGHT, ILI9341_COLOR(255, 0, 0));
    ili9341_fill_rect(bar_width, 0, bar_width, HEIGHT, ILI9341_COLOR(0, 255, 0));
    ili9341_fill_rect(2 * bar_width, 0, bar_width, HEIGHT, ILI9341_COLOR(0, 0, 255));
    ili9341_fill_rect(3 * bar_width, 0, bar_width, HEIGHT, ILI9341_COLOR(255, 255, 255));
    ili9341_text("Color Bars", 20, 20, ILI9341_COLOR(0, 0, 0));
    ili9341_show();
    sleep_ms(2000);

    // Test 4: Default font text
    ili9341_fill(0);
    ili9341_text("Default Font", 20, 20, ILI9341_COLOR(255, 255, 255));
    ili9341_text("8x8 Font", 20, 36, ILI9341_COLOR(255, 255, 0));
    ili9341_text("Version 1.0.4", 20, 52, ILI9341_COLOR(255, 0, 0));
    ili9341_show();
    sleep_ms(2000);

    // Test 5: Single word
    ili9341_fill(0);
    ili9341_text("TEST", 96, 96, ILI9341_COLOR(255, 255, 255));
    ili9341_show();
    sleep_ms(2000);
}

int main() {
    stdio_init_all();
    spi_init(SPI_PORT, 20000000);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    ili9341_init(SPI_CS, SPI_DC, SPI_RST);

    test_display();
    return 0;
}