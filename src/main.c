// General information on our board:
// https://tech.microbit.org/hardware/schematic/

// TODO: Add battery voltage sensing based on https://github.com/microbit-foundation/microbit-v2-battery-voltage

#include <device.h>
#include <devicetree.h>
// The device tree for our nrf52833 is defined in the file
// ~/.platformio/packages/framework-zephyr/dts/arm/nordic/nrf52833.dtsi and nrf52833_qiaa.dts
// The device tree for our micro:bit v2 board is defined in the file
// ~/.platformio/packages/framework-zephyr/boards/arm/bbc_microbit_v2/bbc_microbit_v2.dts
// We locally ovveride the device tree (or at least add to it) in the file
// zephyr/app.overlay
// Note that we could also be more specific by using specific board overlays, but fur us this is sufficient
// For more information on the device trees, see:
// https://docs.nordicsemi.com/bundle/ncs-latest/page/zephyr/build/dts/index.html

#include <sys/printk.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/dis.h>

#include <drivers/pwm.h>

#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <settings/settings.h>

// We use the version.py script to generate a version.h file that contains the current git tag or commit hash
// The script is executed before the pio build process starts, so that the version.h file is available during compilation
#include "version.h"
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "UNDEFINED"
#endif

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// Servo stuff:

// Servo configuration (see the device tree file for the pins)
// The servos are connected to the PWM controller, which is defined in the app.overlay file
// in the zephyr directory.
#define SERVO1_PIN DT_PROP(DT_NODELABEL(pwm0), ch0_pin)
#define SERVO2_PIN DT_PROP(DT_NODELABEL(pwm0), ch1_pin)
#define PWM_PERIOD_USEC 20000U // 20 ms pwm frame
static const struct device *pwm_dev;

// Convert –100...+100% “speed” to 1000...2000 µs pulse
static inline uint32_t speed_to_usec(int8_t speed) {
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;
    // 1500 µs is the stop position, 1000 µs is full reverse, 2000 µs is full forward
    return 1500U + (int32_t)speed * 5U;
}

static void servo_set_speed(uint32_t pin, int8_t speed) {
    uint32_t pulse = speed_to_usec(speed);
    int err = pwm_pin_set_usec(pwm_dev, pin, PWM_PERIOD_USEC, pulse, PWM_POLARITY_NORMAL);
    if (err) {
        printk("PWM set failed on pin %u (err %d)\n", pin, err);
    }
}

// We will later setup the characteristic in a way that both servos are controlled through one characteristic,
// For this, we need a callback function that takes a speed value and applies it to both servos
// 0x64 -> 100% forward, 0x00 -> stop, 0x9C -> 100% reverse
// Note that as the two servos are mirrors of each other, the sender is responsible for sending the correct speed values
// In nordic app, the Byte Array write can be used to send the data, e.g. [0x9c, 0x64] to drive both servos forward
static ssize_t write_servos(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {

    // We expect the buffer to contain two int8_t values, one for each servo
    if (len != 2) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    const int8_t *speeds = buf;
    // Set the speed for both servos
    servo_set_speed(SERVO1_PIN, speeds[0]);
    servo_set_speed(SERVO2_PIN, speeds[1]);
    return 2;
}

// IMU stuff:

// LSM303AGR device drivers exist for both accelerometer and magnetometer, the temperature sensor needs to be accessed directly via I2C (or a custom driver)
// Note that these drivers are defined in the device tree (.dts) file of out micro:bit board. This and additional config files are located here:
// ~/.platformio/packages/framework-zephyr/boards/arm/bbc_microbit_v2
#define ACCEL_NODE DT_INST(0, st_lsm303agr_accel)
#define MAG_NODE DT_INST(0, st_lsm303agr_magn)

// To communicate with the temperature sensor, we use the I2C device
#define I2C_NODE DT_BUS(MAG_NODE)

#define LSM303AGR_ADDR DT_REG_ADDR(ACCEL_NODE)

// LSM303AGR datasheet:
// https://www.st.com/resource/en/datasheet/lsm303agr.pdf

// The LSM303AGR has an internal temperature sensor
// To enable temperature reading, the TEMP_EN[1:0] need to be set in the TEMP_CFG_A register and CTRL_REG4_A.BDU must be set to 1
#define REG_TEMP_CFG_A 0x1F
#define REG_CTRL_REG4_A 0x23
// Temperature data is stored inside OUT_TEMP_H as two’s complement data in 8-bit format with 25°C offset
#define REG_OUT_TEMP_L 0x0C
#define REG_OUT_TEMP_H 0x0D

// Bitmasks for the used operations
#define TEMP_EN_MASK (BIT(7) | BIT(6)) // TEMP_EN[1:0] = 11b to enable temperature sensor
#define ADC_PD_MASK BIT(1)             // ADC_PD = 0 to enable ADC
#define BDU_MASK BIT(7)                // BDU = 1 to enable block data update

// Storage for our IMU data
static int16_t accel3d[3];
static int16_t mag3d[3];
static int16_t temp_c100; // We need to use a 100x factor for fixed point representation, as this is how the temperature characteristic is defined in the Bluetooth specification

// Device driver handles for our sensors
// See https://academy.nordicsemi.com/courses/nrf-connect-sdk-intermediate/lessons/lesson-7-device-driver-dev/topic/device-driver-model-2/
// for more details on the device driver model in Zephyr.
static const struct device *accel_dev, *mag_dev, *i2c_dev;

// Rounding helper
static inline int16_t round_s16(double v) { return (int16_t)(v >= 0 ? v + 0.5 : v - 0.5); }

// Neopixel stuff:

extern void ws2812_loop(const uint8_t *buf, int len); // This is implemented in assembly in the ws2812_loop.S file and is responsible for bit-banging the WS2812 protocol

#define LED_COUNT 5
#define STRIP_BYTES (LED_COUNT * 3)

static ssize_t write_leds(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    printk("write_leds called with len %d and data: ", len);
    for (int i = 0; i < len; i++) {
        printk("%02x ", ((const uint8_t *)buf)[i]);
    }
    printk("\n");
    if (len != STRIP_BYTES) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    // Call the ws2812_loop function to send the data to the neopixels
    ws2812_loop(buf, STRIP_BYTES);
    return STRIP_BYTES;
}

// Bluetooth stuff:

// As we do not commercially use this as a product, we can theoretically choose any UUID we want. But to illustrate the construction of such a UUID,
// here we used the Bluetooth SIG base UUID as starting point and make modifications.
// Commercial uses must be registered with the Bluetooth SIG and use a UUID from the assigned numbers list.
// First, a little info about UUIDs in general:
//  UUIDs (Universally Unique Identifiers) are 128-bit identifiers, which are usually represented as 32 hexadecimal digits,
//  Lets take a UUID like "xxxxxxxx-xxxx-Mxxx-Nxxx-xxxxxxxxxxxx"
//  Here, M indicates the UUID version, N the UUID variant, which should be between 8 and b to indicate RFC 4122/DCE 1.1 UUIDs
//  (see https://volkandogan.net/uuid-guid-versions-and-variants-generating-uuid-examples/ for more information).
// So: Within https://www.bluetooth.com/specifications/assigned-numbers/, all relevant UUIDs are defined. To set a fitting UUID for us, we use the base UUID
// and add the specific service UUID at correct position. The base UUID is defined in the Bluetooth specification (section 2.5.1 in https://www.bluetooth.com/specifications/specs/core-specification-5-3/),
// here is also a nice post about it: https://stackoverflow.com/questions/13964342/android-how-do-bluetooth-uuids-work
//
// So, to select a UUID for our service, we can follow these steps:
// 1. Take the Bluetooth SIG base service UUID: 00000000-0000-1000-8000-00805F9B34FB
// 2. Go to section "3.4 GATT Services" of the Bluetooth SIG assigned numbers list and find a service that fits to our use case, write down the UUID
// 3. Replace the first 8 bytes of the base UUID with the service UUID
//

// Service definitions:
// In our case, we use a "Device Information Service" (DIS) UUID for general information like firmware version and hardware description
// For this we use the predefined BT_UUID_DIS_VAL, which is defined in the Bluetooth specification
// Here is our DIS data:
// TODO: Use this in DIS
static const char *manufacturer = "TU Vienna, Institute of Computer Engineering, Research Group Automation Systems";
static const char *model_number = "Micro:bit-v2 Robot";
static const char *hw_revision = "Rev A";
static const char *fw_revision = FIRMWARE_VERSION; // from version.h
static const char *sw_revision = "Zephyr";         // TODO: Add version from Zephyr

// A custom UUID for a general Robot Control Service (RCS) that we use to control the servos, neopixels and publish accelerometer, magnetometer and temperature data
#define BT_UUID_RCS_VAL 0xff00
#define BT_UUID_RCS BT_UUID_DECLARE_16(BT_UUID_RCS_VAL)

// Characteristic definitions:
// As accerometer is not defined per default, we define it here
#define BT_UUID_ACCEL_3D BT_UUID_DECLARE_16(0xff01)

// Servo control characteristics
#define BT_UUID_SRV_CONTROL BT_UUID_DECLARE_16(0xff02)

// Neopixel control characteristics
#define BT_UUID_LED_CONTROL BT_UUID_DECLARE_16(0xff03)

// As described in https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-4-bluetooth-le-data-exchange/topic/attribute-table/
// A GATT service is a collection of attributes that define the service itself and its characteristics.
// So here we define our GATT service with three characteristics:
// 1. Accelerometer (3x sint16, units = mm/s²)
// 2. Magnetometer (3x sint16, units = µT * 0.1)
// 3. Temperature (sint16, units = °C * 0.01)
// Note that the unites are defined in the Bluetooth specification, see
// https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/generic-attribute-profile--gatt-.html
// for more details. All have a CCC (Client Characteristic Configuration) descriptor to allow notifications
// Note that by using BT_GATT_CHRC_INDICATE instead of BT_GATT_CHRC_NOTIFY, we could also use indications instead of notifications,
// with the difference that the client has to acknowledge the indications

// The DIS service:
BT_GATT_SERVICE_DEFINE(dis_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS));

// The RCS service:
BT_GATT_SERVICE_DEFINE(rcs_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_RCS),

                       // Accelerometer (3x sint16, units = mm/s²)
                       BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_3D, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, accel3d),
                       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       // Magnetometer (3x sint16, units = µT * 0.1)
                       BT_GATT_CHARACTERISTIC(BT_UUID_MAGN_FLUX_DENSITY_3D, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, mag3d),
                       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       // Temperature (sint16, units = °C * 0.01)
                       BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, &temp_c100),
                       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       // Servo control (2x int8_t, units = %, with -100% = full reverse, 0% = stop, +100% = full forward)
                       BT_GATT_CHARACTERISTIC(BT_UUID_SRV_CONTROL, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_servos, NULL),

                       // Neopixel control (5x3x uint8_t, units = RGB, with 0x00 = off, 0xff = full brightness)
                       BT_GATT_CHARACTERISTIC(BT_UUID_LED_CONTROL, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, write_leds, NULL), );

// helper: split a 16‑bit UUID into little‑endian bytes
#define UUID16_BYTES(u) ((u)&0xFF), (((u) >> 8) & 0xFF)
// Here we define our advertising data. Note that we include the complete local name and the service UUID,
// so that we can later find our device by name or UUID (later used in Python script and ROS2 node).
// For more details on advertising, see:
// https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-3-bluetooth-le-connections/topic/connection-process/
// And on connection parameters:
// https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-3-bluetooth-le-connections/topic/connection-parameters/
static const struct bt_data ad[] = {
    // We use general discoverable mode and no BR/EDR support, as our chip only supports BLE
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // Complete local name, so that we can find our device by name
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    // Services
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, UUID16_BYTES(BT_UUID_DIS_VAL), UUID16_BYTES(BT_UUID_RCS_VAL)),
};

// We use workqueues to periodically sample the sensors and notify the clients (a feature we get through the RTOS kernel)
// See https://docs.nordicsemi.com/bundle/ncs-latest/page/zephyr/kernel/services/threads/workqueue.html
// and https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/lessons/lesson-7-multithreaded-applications/topic/scheduler/
static struct k_work_delayable imu_work;

// This is the work handler that is called periodically to sample the sensors and notify the clients
static void imu_work_handler(struct k_work *work) {
    struct sensor_value sa[3], sm[3];
    double tmp;

    // 1) First, we fetch the samples from the sensors
    sensor_sample_fetch(accel_dev);
    sensor_sample_fetch(mag_dev);
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, sa);
    sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, sm);

    // 2) Then, we convert the sensor values to our fixed point representation
    for (int i = 0; i < 3; i++) {
        // Note that wile in the datasheet, it says the unit is in g, but we found that it actually returns the values in mm/s²,
        // probably this is hidden in the device driver implementation
        accel3d[i] = round_s16(sensor_value_to_double(&sa[i]) * 1000.0);
        // Also, the datasheet says gauss, but we found that it actually returns the values in µT * 0.1, again device driver?
        mag3d[i] = round_s16(sensor_value_to_double(&sm[i]) * 1000.0);

        // Uncomment the following line to print the values to the console for debugging
        // printk("Accel %d: %d mm/s², Mag %d: %d µT\n", i, accel3d[i], i, mag3d[i]);
    }

    // 3) As we do not have device driver for the temperature sensor, we access it directly via I2C
    // We read the temperature from the LSM303AGR sensor, which is a two's complement value with a 25°C offset
    uint8_t lo, hi, reg;
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_TEMP_CFG_A, &reg);
    reg = (reg | TEMP_EN_MASK) & ~ADC_PD_MASK;
    i2c_reg_write_byte(i2c_dev, LSM303AGR_ADDR, REG_TEMP_CFG_A, reg);
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_CTRL_REG4_A, &reg);
    i2c_reg_write_byte(i2c_dev, LSM303AGR_ADDR, REG_CTRL_REG4_A, reg | BDU_MASK);
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_OUT_TEMP_L, &lo);
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_OUT_TEMP_H, &hi);
    // hi needs to be cast to a signed int, as it is a two's complement value
    tmp = (double)(int8_t)hi + 25.0 + (lo / 256.0); // We only have 0.25°C resolution as in normal mode, only 3 bits are used. The documentation in the datasheet is abit lacking for temperature
    temp_c100 = round_s16(tmp * 100.0);

    // 4) Finally, we notify the clients about the new values
    // first 3* 2 bytes are for the accelerometer, next 3*2 bytes for the magnetometer and last 2 bytes for the temperature
    bt_gatt_notify(NULL, &rcs_svc.attrs[1], accel3d, sizeof(accel3d));
    bt_gatt_notify(NULL, &rcs_svc.attrs[4], mag3d, sizeof(mag3d));
    bt_gatt_notify(NULL, &rcs_svc.attrs[7], &temp_c100, sizeof(temp_c100));

    // 5) Reschedule the work to run again after 100 milliseconds
    k_work_schedule(&imu_work, K_MSEC(100));
}

static const char firmware_version[] = FIRMWARE_VERSION;

static void bt_ready(int err) {
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    printk("Firmware version: %s\n", firmware_version);

    // Here we advertise, note that the second part would be for scan responses, but for our application, the data in ad suffices
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed (err %d)\n", err);
        return;
    }
    printk("Advertising started\n");

    // This is how to initialize workqueues in Zephyr
    k_work_init_delayable(&imu_work, imu_work_handler);
    // We start the first work after 1 second
    k_work_schedule(&imu_work, K_SECONDS(1));
}

void main(void) {
    k_sleep(K_SECONDS(3)); // Wait for the system to stabilize
    // Retrieve the device handles for our sensors
    accel_dev = DEVICE_DT_GET(ACCEL_NODE);
    mag_dev = DEVICE_DT_GET(MAG_NODE);
    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    pwm_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pwm0)));
    if (!pwm_dev) {
        printk("Error: PWM0 device not found\n");
        return;
    }

    if (!device_is_ready(accel_dev) || !device_is_ready(mag_dev) || !device_is_ready(i2c_dev)) {
        printk("Device not ready\n");
        return;
    }

    // For NeoPixels, we need to set the GPIO pins to output mode
    const struct device *gpio0 = device_get_binding("GPIO_0");

    if (!gpio0) {
        printk("Error: GPIO_0 device not found\n");
        return;
    }

    gpio_pin_configure(gpio0, 2, GPIO_OUTPUT_LOW); // P0.02 data out

    int rc = bt_enable(bt_ready);
    if (rc) {
        printk("bt_enable failed (err %d)\n", rc);
    }
}
