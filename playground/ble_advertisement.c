#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/gpio.h>

#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define LED_ROW_PIN    21
#define LED_COL_PIN    28

static const struct device *gpio_dev;

// GATT write handler
ssize_t led_write_cb(struct bt_conn *conn,
                     const struct bt_gatt_attr *attr,
                     const void *buf, uint16_t len,
                     uint16_t offset, uint8_t flags)
{
    if (len >= 1) {
        uint8_t v = ((uint8_t *)buf)[0];
        bool on = (v == '1');

        gpio_pin_set(gpio_dev, LED_ROW_PIN, on ? 1 : 0);
        gpio_pin_set(gpio_dev, LED_COL_PIN, on ? 0 : 1);

        printk("LED set to %s\n", on ? "ON" : "OFF");
    }

    return len;
}

// Custom service UUIDs
#define BT_UUID_LED_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x9abcdef0f0f0)

#define BT_UUID_LED_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x9abcdef0f0f1)

static struct bt_uuid_128 led_service_uuid = BT_UUID_INIT_128(BT_UUID_LED_SERVICE_VAL);
static struct bt_uuid_128 led_char_uuid    = BT_UUID_INIT_128(BT_UUID_LED_CHAR_VAL);

BT_GATT_SERVICE_DEFINE(led_svc,
    BT_GATT_PRIMARY_SERVICE(&led_service_uuid),
    BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, led_write_cb, NULL),
);

// Advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    settings_load();  // Load identity

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    };

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed (err %d)\n", err);
    } else {
        printk("Advertising started successfully\n");
    }
}


void main(void)
{
    printk("Booting BLE LED toggle demo\n");

    gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
    __ASSERT(gpio_dev, "GPIO0 not found");

    gpio_pin_configure(gpio_dev, LED_ROW_PIN, GPIO_OUTPUT);
    gpio_pin_configure(gpio_dev, LED_COL_PIN, GPIO_OUTPUT);

    int err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable failed (err %d)\n", err);
    }
}
