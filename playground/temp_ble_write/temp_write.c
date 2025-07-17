#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <devicetree.h>

/* BLE device name from Kconfig */
#define DEVICE_NAME        CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN    (sizeof(DEVICE_NAME) - 1)

/* LSM303AGR nodes & registers */
#define ACCEL_NODE         DT_INST(0, st_lsm303agr_accel)
#define MAG_NODE           DT_INST(0, st_lsm303agr_magn)
#define I2C_NODE           DT_BUS(ACCEL_NODE)
#define LSM303AGR_ADDR     DT_REG_ADDR(ACCEL_NODE)
#define REG_TEMP_CFG_A     0x1F
#define REG_CTRL_REG4_A    0x23
#define REG_OUT_TEMP_L     0x0C
#define REG_OUT_TEMP_H     0x0D
#define TEMP_EN_MASK       (BIT(7) | BIT(6))
#define ADC_PD_MASK        BIT(1)
#define BDU_MASK           BIT(7)

/* If you ever need custom 128-bit UUIDs, wrap them in a proper block comment:
   /*
   #define BT_UUID_TEMP_SERVICE_VAL \
       BT_UUID_128_ENCODE(0xabcdef01, 0x2345, 0x6789, 0xabcd, 0xef0123456789)
   #define BT_UUID_TEMP_CHAR_VAL \
       BT_UUID_128_ENCODE(0xabcdef01, 0x2345, 0x6789, 0xabcd, 0xef0123456790)
   *\/
*/

/* 16-bit UUID for Environmental Sensing Service (0x181A) */
static struct bt_uuid_16 ess_svc_uuid =
    BT_UUID_INIT_16(BT_UUID_ESS_VAL);

/* 16-bit UUID for Temperature Characteristic (0x2A6E) */
static struct bt_uuid_16 temp_char_uuid =
    BT_UUID_INIT_16(BT_UUID_TEMPERATURE_VAL);

/* Current temperature value, in hundredths of a degree Celsius */
static int16_t current_temp_hundredths;

/* CCC callback state */
static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Temperature notifications %s\n",
           notif_enabled ? "enabled" : "disabled");
}

/* GATT attribute table using the Environmental Sensing Service */
BT_GATT_SERVICE_DEFINE(env_svc,
    BT_GATT_PRIMARY_SERVICE(&ess_svc_uuid),

    /* Temperature characteristic: read + notify */
    BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid,
                           BT_GATT_CHRC_READ  | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, /* read callback */
                           NULL, /* write callback */
                           &current_temp_hundredths),

    BT_GATT_CCC(temp_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Advertising data (general discoverable + no BR/EDR) */
//! See: https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-2-bluetooth-le-advertising/topic/advertisement-packet/
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    /* Advertise 16-bit service UUID 0x181A (little-endian: 0x1A, 0x18) */
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  0x1A, 0x18),
};

/* Scan response data: complete device name */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Work and timer to sample & notify */
static struct k_work_delayable temp_work;
static const struct device *i2c_dev;

static void temp_work_handler(struct k_work *work)
{
    uint8_t reg, lo, hi;
    int16_t raw;
    int deg, frac;

    /* 1) Power up ADC and enable block data update (BDU) */
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_TEMP_CFG_A, &reg);
    reg = (reg | TEMP_EN_MASK) & ~ADC_PD_MASK;
    i2c_reg_write_byte(i2c_dev, LSM303AGR_ADDR, REG_TEMP_CFG_A, reg);

    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_CTRL_REG4_A, &reg);
    reg |= BDU_MASK;
    i2c_reg_write_byte(i2c_dev, LSM303AGR_ADDR, REG_CTRL_REG4_A, reg);

    /* 2) Wait for conversion */
    k_sleep(K_MSEC(100));

    /* 3) Read raw temperature */
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_OUT_TEMP_L, &lo);
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_OUT_TEMP_H, &hi);
    raw = (int16_t)((hi << 8) | lo);

    /* 4) Convert to hundredths of °C */
    deg  = (raw >> 8) + 25;
    frac = ((raw & 0xFF) * 100 + 128) / 256;
    if (frac == 100) {
        deg++;
        frac = 0;
    }
    current_temp_hundredths = deg * 100 + frac;

    /* 5) Notify if enabled, using env_svc */
    bt_gatt_notify(NULL, &env_svc.attrs[1],
                   &current_temp_hundredths,
                   sizeof(current_temp_hundredths));

    /* 6) Re-queue for 1s later */
    k_work_schedule(&temp_work, K_SECONDS(1));
}

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
    settings_load();

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed (err %d)\n", err);
    } else {
        printk("Advertising successfully started\n");
        /* Kick off our periodic temperature work */
        k_work_init_delayable(&temp_work, temp_work_handler);
        k_work_schedule(&temp_work, K_SECONDS(1));
    }
}

void main(void)
{
    int err;

    printk("Booting BLE EnvSense Temp Demo\n");

    /* Bind I2C */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    /* Enable BT and start */
    err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable failed (err %d)\n", err);
    }
}
