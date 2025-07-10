#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#define NUM_ROWS 5
#define NUM_COLS 5

// Pin definitions for micro:bit V2 LED matrix:
static const uint8_t row_pins[NUM_ROWS]  = {21, 22, 15, 24, 19};
static const uint8_t row_ports[NUM_ROWS] = { 0,  0,  0,  0,  0 };

static const uint8_t col_pins[NUM_COLS]  = {28, 11, 31,  5, 30};
static const uint8_t col_ports[NUM_COLS] = { 0,  0,  0,  1,  0 };

static const struct device *gpio0_dev;
static const struct device *gpio1_dev;

// Turn all LEDs off (high-Z on every pin)
static void matrix_off(void)
{
    for (int i = 0; i < NUM_ROWS; i++) {
        const struct device *dev = row_ports[i] ? gpio1_dev : gpio0_dev;
        gpio_pin_configure(dev, row_pins[i], GPIO_INPUT);
    }
    for (int i = 0; i < NUM_COLS; i++) {
        const struct device *dev = col_ports[i] ? gpio1_dev : gpio0_dev;
        gpio_pin_configure(dev, col_pins[i], GPIO_INPUT);
    }
}

static void matrix_set(uint8_t r, uint8_t c, bool on)
{
    matrix_off();
    if (!on) {
        return;
    }
    
    const struct device *rdev = row_ports[r] ? gpio1_dev : gpio0_dev;
    gpio_pin_configure(rdev, row_pins[r], GPIO_OUTPUT_HIGH);
    const struct device *cdev = col_ports[c] ? gpio1_dev : gpio0_dev;
    gpio_pin_configure(cdev, col_pins[c], GPIO_OUTPUT_LOW);
}

void main(void)
{
    // Bind both GPIO devices
    gpio0_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
    gpio1_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio1)));
    if (!gpio0_dev || !gpio1_dev) {
        printk("ERROR: GPIO0 or GPIO1 not found\n");
        return;
    }

    // Init all to off
    for (int i = 0; i < NUM_ROWS; i++) {
        const struct device *dev = row_ports[i] ? gpio1_dev : gpio0_dev;
        gpio_pin_configure(dev, row_pins[i], GPIO_OUTPUT_LOW);
    }
    for (int i = 0; i < NUM_COLS; i++) {
        const struct device *dev = col_ports[i] ? gpio1_dev : gpio0_dev;
        gpio_pin_configure(dev, col_pins[i], GPIO_OUTPUT_HIGH);
    }

    while (1) {
        for (uint8_t r = 0; r < NUM_ROWS; r++) {
            for (uint8_t c = 0; c < NUM_COLS; c++) {
                matrix_set(r, c, true);
                k_msleep(500);
                matrix_set(r, c, false);
                k_msleep(100);
            }
        }
    }
}