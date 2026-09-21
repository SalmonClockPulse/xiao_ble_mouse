#include "zephyr/bluetooth/uuid.h"
#include <stdint.h>
#include <sys/cdefs.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/gatt.h>

#define STACK_SIZE 1024
#define PRIORITY 7

struct sensor_data {
    struct sensor_value x;
    struct sensor_value y;
    struct sensor_value z;
};

struct mouse_report {
    uint8_t buttons;
    int8_t x;
    int8_t y;
};

static const uint8_t report_desc[] = {
    0x05, 0x01, // Usage Page (Generic desktop)
    0x09, 0x02, // Usage (Mouse)
    0xa1, 0x01, // Collection (Application)
    0x09, 0x01, // Usage (Pointer)
    0xa1, 0x00, // Collection (Physical)
    0x05, 0x09, // Usage Page (Button)
    0x19, 0x01, // Usage Minimum (1)
    0x29, 0x03, // Usage Maximum (3)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x75, 0x01, // Report Size (1)
    0x95, 0x03, // Report Count (3)
    0x81, 0x02, // Input (Data, Var, Abs)
    0x75, 0x05, // Report Size (5)
    0x95, 0x01, // Report Count (1)
    0x81, 0x01, // Input (Cnst, Arr, Abs)
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x30, // Usage (X)
    0x09, 0x31, // Usage (Y)
    0x09, 0x38, // Usage (Wheel)
    0x15, 0x81, // Logical Minimum (-127)
    0x25, 0x7f, // Logical Maximum (127)
    0x75, 0x08, // Report Size (8)
    0x95, 0x03, // Report Count (3)
    0x81, 0x06, // Input (Data, Var, Rel)
    0xc0,       // End Collection
    0xc0,       // End Collection
};

struct hids_info {
    uint16_t version;
    uint8_t code;
    uint8_t flags;
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = BIT(1)
};

static ssize_t read_info(
    struct bt_conn *conn,
    const struct bt_gatt_attr *attr, void *buf,
    uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(
        conn, attr, buf, len, offset, attr->user_data,
        sizeof(struct hids_info));
}

static ssize_t read_report_map(
    struct bt_conn *conn, const struct bt_gatt_attr *attr,
    void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(
        conn, attr, buf, len, offset, report_desc, sizeof(report_desc));
}

static ssize_T read_input_report()

BT_GATT_SERVICE_DEFINE(hog_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ, read_info, NULL, &info),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ, read_report_map, NULL, NULL),
    BT_GATT_CHARACTERISTIC(
        BT_UUID_HIDS_INFO,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT, read_input_report, NULL, NULL),
    )
);

K_MSGQ_DEFINE_STATIC(gyro_value, sizeof(struct sensor_data), 10, 4);

static const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

void get_lsm6dsl_value()
{
    struct sensor_value odr_attr;
    struct sensor_data gyro_set;

    if (!device_is_ready(lsm6dsl_dev)) {
        printk("sensor: device is not ready yet.\n");
        return;
    }

    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(
        lsm6dsl_dev,
        SENSOR_CHAN_GYRO_XYZ,
        SENSOR_ATTR_SAMPLING_FREQUENCY,
        &odr_attr) < 0) {
            printk("Could not set sensor type and channel.\n");
            return;
        }
    while(1) {
        sensor_sample_fetch(lsm6dsl_dev);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &gyro_set.x);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &gyro_set.y);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Z, &gyro_set.z);
        printk("gyro_x: %d.%d gyro_y: %d.%d gyro_z:%d.%d\n",
            gyro_set.x.val1, gyro_set.x.val2,
            gyro_set.y.val1, gyro_set.y.val2,
            gyro_set.z.val1, gyro_set.z.val2);
    }

    k_msgq_put(&gyro_value, &gyro_set, K_NO_WAIT);
}

void send_hid_report()
{
    struct mouse_report mouse_map;
    while(1){
        bt_gatt_notify(NULL, &hog_svc.attrs[5], &mouse_map, sizeof(struct mouse_report));
        k_sleep(K_MSEC(100));
    }
}

K_THREAD_DEFINE(get_lsm6dsl_value_id, STACK_SIZE, get_lsm6dsl_value, NULL, NULL, NULL, PRIORITY, 0, 0);
