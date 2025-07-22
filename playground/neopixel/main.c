// src/main.c
#include <device.h>
#include <drivers/gpio.h>
#include <string.h>
#include <zephyr.h>

extern void ws2812_loop(const uint8_t *buf, int len);

#define LED_COUNT 5
#define STRIP_BYTES (LED_COUNT * 3)
#define DELAY_MS 500

void main(void) {
    const struct device *gpio0 = device_get_binding("GPIO_0");
    __ASSERT(gpio0, "GPIO_0 not found");
    gpio_pin_configure(gpio0, 2, GPIO_OUTPUT_LOW); // P0.02 data out
    k_msleep(100);                                 // allow strip to power up

    uint8_t strip[STRIP_BYTES];

    while (1) {
        // ── GREEN ──
        for (int i = 0; i < LED_COUNT; i++) {
            strip[3 * i + 0] = 0xFF; // G
            strip[3 * i + 1] = 0x00; // R
            strip[3 * i + 2] = 0x00; // B
        }
        ws2812_loop(strip, STRIP_BYTES);
        k_msleep(DELAY_MS);
    }
}
