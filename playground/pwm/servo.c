/* main.c */

#include <device.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <zephyr.h>

#define SERVO1_PIN 3           /* micro:bit P1 → P0.03 */
#define SERVO2_PIN 4           /* micro:bit P2 → P0.04 */
#define PWM_PERIOD_USEC 20000U /* 20 ms frame */

/* PWM device binding */
static const struct device *pwm_dev;

/* Convert –100…+100% speed to 1000…2000 µs pulse (1500 µs = stop) */
static inline uint32_t speed_to_usec(int8_t speed) {
    if (speed > 100) {
        speed = 100;
    } else if (speed < -100) {
        speed = -100;
    }
    return 1500U + (int32_t)speed * 5U;
}

/* Apply a speed command to one servo channel */
static void servo_set_speed(uint32_t pin, int8_t speed) {
    uint32_t pulse = speed_to_usec(speed);
    int err = pwm_pin_set_usec(pwm_dev, pin, PWM_PERIOD_USEC, pulse, PWM_POLARITY_NORMAL);
    if (err) {
        printk("Failed to set PWM on pin %u (err %d)\n", pin, err);
    }
}

void main(void) {
    k_msleep(2000);

    /* Look up the Zephyr PWM controller named "PWM_0" in devicetree */
    pwm_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pwm0)));
    if (!pwm_dev) {
        printk("Error: PWM0 device not found\n");
        return;
    }

    printk("Servo demo starting\n");

    while (1) {
        /* Forward at 50% */
        servo_set_speed(SERVO1_PIN, 50);
        servo_set_speed(SERVO2_PIN, 50);
        k_msleep(2000);

        /* Stop */
        servo_set_speed(SERVO1_PIN, 0);
        servo_set_speed(SERVO2_PIN, 0);
        k_msleep(1000);

        /* Reverse at 75% */
        servo_set_speed(SERVO1_PIN, -75);
        servo_set_speed(SERVO2_PIN, -75);
        k_msleep(2000);

        /* Stop again */
        servo_set_speed(SERVO1_PIN, 0);
        servo_set_speed(SERVO2_PIN, 0);
        k_msleep(1000);
    }
}
