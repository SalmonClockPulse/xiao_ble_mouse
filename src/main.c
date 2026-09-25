#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>

#include "ble_hid.c"

#define STACK_SIZE 1024
#define PRIORITY 7

static const struct gpio_dt_spec cursor_enable_button = GPIO_DT_SPEC_GET(DT_ALIAS(cursor_enable_button), gpios);

struct sensor_data {
    struct sensor_value x;
    struct sensor_value y;
    struct sensor_value z;
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    printk("Passkey for %s: %06u\n", bt_conn_dst_str(conn), passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
    printk("Pairing cancelled: %s\n", bt_conn_dst_str(conn));
}

static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel
};

K_MSGQ_DEFINE_STATIC(gyro_value, sizeof(struct sensor_data), 10, 4);

static const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

int8_t sensor_value2int8(const struct sensor_value *val, int32_t scale)
{
    int64_t micro = (int64_t)val->val1 * 1000000LL + val->val2;
    int64_t scaled = (micro * scale) / 1000000LL;

    if (scaled > 127) return 127;
    else if (scaled < -128) return -128;
    return (int8_t)scaled;
}

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
        k_msgq_put(&gyro_value, &gyro_set, K_NO_WAIT);
        //printk("gyro_x: %d.%d gyro_y: %d.%d gyro_z:%d.%d\n",
        //    gyro_set.x.val1, gyro_set.x.val2,
        //    gyro_set.y.val1, gyro_set.y.val2,
        //    gyro_set.z.val1, gyro_set.z.val2);
    }
}

void send_hid_report()
{
    int err;
    const int32_t scale = 10;
    struct sensor_data gyro_get;

    gpio_pin_configure_dt(&cursor_enable_button, GPIO_INPUT);

    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SAMPLE_BT_USE_AUTHENTICATION)) {
        bt_conn_auth_cb_register(&auth_cb_display);
        printk("Bluetooth authentication callbacks registered.\n");
    }
    while(1){
        k_msgq_get(&gyro_value, &gyro_get, K_FOREVER);
        int is_cursor = gpio_pin_get_dt(&cursor_enable_button);
        if (is_cursor == 0) {
            gyro_get.y.val1 = -gyro_get.y.val1;
            gyro_get.y.val2 = -gyro_get.y.val2;
            gyro_get.z.val1 = -gyro_get.z.val1;
            gyro_get.z.val2 = -gyro_get.z.val2;
            send_mouse_report(
                sensor_value2int8(&gyro_get.z, scale),
                sensor_value2int8(&gyro_get.y, scale),0,0);
        }
    }
}

K_THREAD_DEFINE(get_lsm6dsl_value_id, STACK_SIZE, get_lsm6dsl_value, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(send_hid_report_id, STACK_SIZE, send_hid_report, NULL, NULL, NULL, PRIORITY, 0, 0);
