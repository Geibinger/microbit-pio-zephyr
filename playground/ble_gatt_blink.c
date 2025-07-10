#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <device.h>
#include <drivers/gpio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/gatt.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static struct device *gpio_dev;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,        BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,   0xAA, 0xFE),
    BT_DATA_BYTES(BT_DATA_SVC_DATA16,   0xAA, 0xFE,
                  0x10, /* URL frame */
                  0x00, /* Tx power */
                  0x00, /* http://www. */
                  'z','e','p','h','y','r',
                  'p','r','o','j','e','c','t',
                  0x08) /* .org */
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* 128‐bit UUIDs */
static struct bt_uuid_128 led_svc_uuid = BT_UUID_INIT_128(
    0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0,
    0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0
);
static struct bt_uuid_128 led_char_uuid = BT_UUID_INIT_128(
    0xAB,0xCD,0xEF,0x01,0x23,0x45,0x67,0x89,
    0xAB,0xCD,0xEF,0x01,0x23,0x45,0x67,0x89
);

static ssize_t write_led(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t v = ((uint8_t *)buf)[0];
    printk("LED write: %d\n", v);
    gpio_pin_set     (gpio_dev, 21, v ? 1 : 0);
    gpio_pin_set     (gpio_dev, 28, 0);
    return len;
}

static struct bt_gatt_attr led_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(&led_svc_uuid),
    BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_led, NULL),
};

static struct bt_gatt_service led_svc = BT_GATT_SERVICE(led_attrs);

// This is called when Bluetooth is starting up
static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    bt_gatt_service_register(&led_svc);

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed (err %d)\n", err);
        return;
    }
    printk("Advertising + GATT service started\n");
}

void main(void)
{
    int err;

    printk("Starting Beacon+LED Demo\n");

    gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
    if (!gpio_dev) {
        printk("Failed to bind GPIO0\n");
        return;
    }
    gpio_pin_configure(gpio_dev, 21, GPIO_OUTPUT);
    gpio_pin_configure(gpio_dev, 28, GPIO_OUTPUT);
    err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable failed (err %d)\n", err);
    }
}
