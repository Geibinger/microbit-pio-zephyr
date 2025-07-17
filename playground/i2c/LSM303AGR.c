/* src/main.c */

#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/printk.h>
#include <zephyr.h>

/* on-board LSM303AGR accel & mag */
#define ACCEL_NODE DT_INST(0, st_lsm303agr_accel)
#define MAG_NODE DT_INST(0, st_lsm303agr_magn)
#define I2C_NODE DT_BUS(ACCEL_NODE)

/* I²C address of the accel/mag */
#define LSM303AGR_ADDR DT_REG_ADDR(ACCEL_NODE)

/* temperature registers on the accel */
#define REG_TEMP_CFG_A 0x1F
#define REG_CTRL_REG4_A 0x23
#define REG_OUT_TEMP_L 0x0C
#define REG_OUT_TEMP_H 0x0D

/* bits to power up the temp ADC and enable BDU */
#define TEMP_EN_MASK (BIT(7) | BIT(6))
#define ADC_PD_MASK BIT(1)
#define BDU_MASK BIT(7)

void main(void) {
    const struct device *accel = DEVICE_DT_GET(ACCEL_NODE);
    const struct device *mag = DEVICE_DT_GET(MAG_NODE);
    const struct device *i2c = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(accel) || !device_is_ready(mag) || !device_is_ready(i2c)) {
        printk("Error: device not ready\n");
        return;
    }

    printk("LSM303AGR accel+mag+temp demo\n");

    while (1) {
        /* 1) Fetch accel & mag via sensor API */
        sensor_sample_fetch(accel);
        sensor_sample_fetch(mag);

        struct sensor_value a[3], m[3];
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, &a[0]);
        sensor_channel_get(mag, SENSOR_CHAN_MAGN_XYZ, &m[0]);

        /* 2) Convert to nice numbers */
        double ax = sensor_value_to_double(&a[0]);
        double ay = sensor_value_to_double(&a[1]);
        double az = sensor_value_to_double(&a[2]);
        double mx = sensor_value_to_double(&m[0]);
        double my = sensor_value_to_double(&m[1]);
        double mz = sensor_value_to_double(&m[2]);

        /* 3) Read raw temperature (same as your working snippet) */
        /* power up ADC */
        uint8_t reg;
        i2c_reg_read_byte(i2c, LSM303AGR_ADDR, REG_TEMP_CFG_A, &reg);
        reg = (reg | TEMP_EN_MASK) & ~ADC_PD_MASK;
        i2c_reg_write_byte(i2c, LSM303AGR_ADDR, REG_TEMP_CFG_A, reg);

        /* enable block-data-update */
        i2c_reg_read_byte(i2c, LSM303AGR_ADDR, REG_CTRL_REG4_A, &reg);
        reg |= BDU_MASK;
        i2c_reg_write_byte(i2c, LSM303AGR_ADDR, REG_CTRL_REG4_A, reg);

        k_sleep(K_MSEC(100)); /* wait for conversion */

        uint8_t lo, hi;
        i2c_reg_read_byte(i2c, LSM303AGR_ADDR, REG_OUT_TEMP_L, &lo);
        i2c_reg_read_byte(i2c, LSM303AGR_ADDR, REG_OUT_TEMP_H, &hi);
        int16_t raw = (int16_t)((hi << 8) | lo);

        /* 4) Print accel & mag (double-precision) */
        printk(">aX: %7.3f\n", ax);
        printk(">aY: %7.3f\n", ay);
        printk(">aZ: %7.3f\n", az);
        printk(">mX: %7.3f\n", mx);
        printk(">mY: %7.3f\n", my);
        printk(">mZ: %7.3f\n", mz);

        /* 5) Print temperature with two-digit integer math */
        int deg = (raw >> 8) + 25;
        int frac = ((raw & 0xFF) * 100 + 128) / 256; /* round */
        if (frac == 100) {
            deg++;
            frac = 0;
        }
        printk(">T: %d.%02d\n", deg, frac);

        k_sleep(K_SECONDS(1));
    }
}
