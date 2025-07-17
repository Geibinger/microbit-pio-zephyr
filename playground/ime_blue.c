#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <settings/settings.h>
#include <sys/printk.h>
#include <zephyr.h>

/* BLE device name from Kconfig */
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* LSM303AGR nodes & registers */
#define ACCEL_NODE DT_INST(0, st_lsm303agr_accel)
#define MAG_NODE DT_INST(0, st_lsm303agr_magn)
#define I2C_NODE DT_BUS(ACCEL_NODE)
#define LSM303AGR_ADDR DT_REG_ADDR(ACCEL_NODE)
#define REG_TEMP_CFG_A 0x1F
#define REG_CTRL_REG4_A 0x23
#define REG_OUT_TEMP_L 0x0C
#define REG_OUT_TEMP_H 0x0D
#define TEMP_EN_MASK (BIT(7) | BIT(6))
#define ADC_PD_MASK BIT(1)
#define BDU_MASK BIT(7)

/* Custom 128‑bit UUIDs */
#define BT_UUID_IMU_SVC_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x1234567890ab)
#define BT_UUID_ACCEL_CHAR_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x1234567890ac)
#define BT_UUID_MAG_CHAR_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x1234567890ad)
#define BT_UUID_TEMP_CHAR_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x1234567890ae)

static struct bt_uuid_128 imu_svc_uuid = BT_UUID_INIT_128(BT_UUID_IMU_SVC_VAL);
static struct bt_uuid_128 accel_char_uuid = BT_UUID_INIT_128(BT_UUID_ACCEL_CHAR_VAL);
static struct bt_uuid_128 mag_char_uuid = BT_UUID_INIT_128(BT_UUID_MAG_CHAR_VAL);
static struct bt_uuid_128 temp_char_uuid = BT_UUID_INIT_128(BT_UUID_TEMP_CHAR_VAL);

/* 16‑bit GATT descriptor UUIDs */
#define BT_UUID_2901 BT_UUID_DECLARE_16(0x2901) /* User Description */
#define BT_UUID_2904 BT_UUID_DECLARE_16(0x2904) /* Presentation Format */

/* CPF structs: describe format/unit/exponent/description for each characteristic */
/* Acceleration – 3D (0x2C1D), unit = gn (0x27C9), signed‑16, exponent = –3 (mg) */
static struct bt_gatt_cpf accel_cpf = {
    .format = BT_GATT_CPF_FORMAT_S16,
    .exponent = -3,
    .unit = 0x27C9,
    .name_space = BT_GATT_CPF_NAMESPACE_BTSIG,
    .description = 0x2C1D,
};

/* Magnetometer – 3D (0x2AA1), unit = microtesla (0x272F), signed‑16, exponent = –1 (0.1 μT) */
static struct bt_gatt_cpf mag_cpf = {
    .format = BT_GATT_CPF_FORMAT_S16,
    .exponent = -1,
    .unit = 0x272F,
    .name_space = BT_GATT_CPF_NAMESPACE_BTSIG,
    .description = 0x2AA1,
};

/* Temperature (0x2A6E), unit = degree Celsius (0x272F), signed‑16, exponent = –2 (raw is °C×100) */
static struct bt_gatt_cpf temp_cpf = {
    .format = BT_GATT_CPF_FORMAT_S16,
    .exponent = -2,
    .unit = 0x272F,
    .name_space = BT_GATT_CPF_NAMESPACE_BTSIG,
    .description = 0x2A6E,
};

/* Current sensor values */
static int16_t current_accel[3];
static int16_t current_mag[3];
static int16_t current_temp; /* in hundredths °C */

/* CCC callback */
static void imu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif = (value == BT_GATT_CCC_NOTIFY);
    printk("IMU notifications %s\n", notif ? "enabled" : "disabled");
}

/* GATT service definition */
BT_GATT_SERVICE_DEFINE(imu_svc,
                       BT_GATT_PRIMARY_SERVICE(&imu_svc_uuid),

                       /* Accelerometer characteristic */
                       BT_GATT_CHARACTERISTIC(&accel_char_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, &current_accel),
                       /* User Description */
                       BT_GATT_DESCRIPTOR(BT_UUID_2901, BT_GATT_PERM_READ, bt_gatt_attr_read_cud, NULL, "Acceleration - 3D"),
                       /* Presentation Format */
                       BT_GATT_DESCRIPTOR(BT_UUID_2904, BT_GATT_PERM_READ, bt_gatt_attr_read_cpf, NULL, &accel_cpf),
                       BT_GATT_CCC(imu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* Magnetometer characteristic */
                       BT_GATT_CHARACTERISTIC(&mag_char_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, &current_mag),
                       BT_GATT_DESCRIPTOR(BT_UUID_2901, BT_GATT_PERM_READ, bt_gatt_attr_read_cud, NULL, "Magnetic Flux Density - 3D"),
                       BT_GATT_DESCRIPTOR(BT_UUID_2904, BT_GATT_PERM_READ, bt_gatt_attr_read_cpf, NULL, &mag_cpf),
                       BT_GATT_CCC(imu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       /* Temperature characteristic */
                       BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, &current_temp),
                       BT_GATT_DESCRIPTOR(BT_UUID_2901, BT_GATT_PERM_READ, bt_gatt_attr_read_cud, NULL, "Temperature"),
                       BT_GATT_DESCRIPTOR(BT_UUID_2904, BT_GATT_PERM_READ, bt_gatt_attr_read_cpf, NULL, &temp_cpf),
                       BT_GATT_CCC(imu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

/* Advertising data: flags + 128‑bit UUID */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

/* Scan response: complete device name */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct k_work_delayable imu_work;
static const struct device *accel_dev, *mag_dev, *i2c_dev;

static void imu_work_handler(const struct k_work *work) {
    struct sensor_value sa[3], sm[3];

    /* 1) Fetch accel/mag */
    sensor_sample_fetch(accel_dev);
    sensor_sample_fetch(mag_dev);
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, sa);
    sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, sm);

    /* 2) Convert to int16: accel in mg, mag in uT */
    for (int i = 0; i < 3; i++) {
        current_accel[i] = (int16_t)(sensor_value_to_double(&sa[i]) * 1000);
        current_mag[i] = (int16_t)(sensor_value_to_double(&sm[i]) * 10);
    }

    /* 3) Read and convert temperature */
    uint8_t lo, hi, reg;
    int16_t raw;
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_TEMP_CFG_A, &reg);
    reg = (reg | TEMP_EN_MASK) & ~ADC_PD_MASK;
    i2c_reg_write_byte(i2c_dev, LSM303AGR_ADDR, REG_TEMP_CFG_A, reg);
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_CTRL_REG4_A, &reg);
    i2c_reg_write_byte(i2c_dev, LSM303AGR_ADDR, REG_CTRL_REG4_A, reg | BDU_MASK);
    k_sleep(K_MSEC(100));
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_OUT_TEMP_L, &lo);
    i2c_reg_read_byte(i2c_dev, LSM303AGR_ADDR, REG_OUT_TEMP_H, &hi);
    raw = (int16_t)((hi << 8) | lo);
    int deg = (raw >> 8) + 25;
    int frac = ((raw & 0xFF) * 100 + 128) / 256;
    if (frac == 100) {
        deg++;
        frac = 0;
    }
    current_temp = deg * 100 + frac;

    /* 4) Notify */
    bt_gatt_notify(NULL, &imu_svc.attrs[1], current_accel, sizeof(current_accel));
    bt_gatt_notify(NULL, &imu_svc.attrs[4], current_mag, sizeof(current_mag));
    bt_gatt_notify(NULL, &imu_svc.attrs[7], &current_temp, sizeof(current_temp));

    /* 5) Reschedule */
    k_work_schedule(&imu_work, K_SECONDS(1));
}

static void bt_ready(int err) {
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
    settings_load();

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed (err %d)\n", err);
        return;
    }
    printk("Advertising successfully started\n");

    k_work_init_delayable(&imu_work, imu_work_handler);
    k_work_schedule(&imu_work, K_SECONDS(1));
}

void main(void) {
    printk("Booting BLE IMU Demo\n");

    accel_dev = DEVICE_DT_GET(ACCEL_NODE);
    mag_dev = DEVICE_DT_GET(MAG_NODE);
    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(accel_dev) || !device_is_ready(mag_dev) || !device_is_ready(i2c_dev)) {
        printk("Device not ready\n");
        return;
    }

    int err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable failed (err %d)\n", err);
    }
}
