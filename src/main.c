#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <math.h>

#include "ble_hid.c"

#define STACK_SIZE 1024
#define PRIORITY 7
#define SENSOR_POLL_INTERVAL_MS 10
#define CURSOR_DEADZONE 0.05

static const struct gpio_dt_spec cursor_enable_button = GPIO_DT_SPEC_GET(DT_ALIAS(cursor_enable_button), gpios);

struct sensor_data {
    struct {
        struct sensor_value x;
        struct sensor_value y;
        struct sensor_value z;
    } gyro_data;
    struct {
        struct sensor_value x;
        struct sensor_value y;
        struct sensor_value z;
    } accel_data;
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

K_MSGQ_DEFINE_STATIC(imu_msgq, sizeof(struct sensor_data), 10, 4);

static const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

void get_lsm6dsl_value()
{
    struct sensor_value odr_attr;
    struct sensor_data imu_set;
    int err;

    if (!device_is_ready(lsm6dsl_dev)) {
        printk("sensor: device is not ready yet.\n");
        return;
    }

    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    err = sensor_attr_set(
        lsm6dsl_dev,
        SENSOR_CHAN_GYRO_XYZ,
        SENSOR_ATTR_SAMPLING_FREQUENCY,
        &odr_attr);
    if (err < 0) {
            printk("Could not set sensor type and channel.\n");
            return;
        }
    err = sensor_attr_set(
        lsm6dsl_dev,
        SENSOR_CHAN_ACCEL_XYZ,
        SENSOR_ATTR_SAMPLING_FREQUENCY,
        &odr_attr);
    if (err < 0) {
            printk("Could not set sensor type and channel.\n");
            return;
        }
    while(1) {
        sensor_sample_fetch(lsm6dsl_dev);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &imu_set.gyro_data.x);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &imu_set.gyro_data.y);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Z, &imu_set.gyro_data.z);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_X, &imu_set.accel_data.x);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Y, &imu_set.accel_data.y);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Z, &imu_set.accel_data.z);
        k_msgq_put(&imu_msgq, &imu_set, K_NO_WAIT);
        //printk("gyro_x: %d.%d gyro_y: %d.%d gyro_z:%d.%d\n",
        //    gyro_set.x.val1, gyro_set.x.val2,
        //    gyro_set.y.val1, gyro_set.y.val2,
        //    gyro_set.z.val1, gyro_set.z.val2);
        //printk("accel_x: %d.%d accel_y: %d.%d accel_z:%d.%d\n",
        //    imu_set.accel_data.x.val1, imu_set.accel_data.x.val2,
        //    imu_set.accel_data.y.val1, imu_set.accel_data.y.val2,
        //    imu_set.accel_data.z.val1, imu_set.accel_data.z.val2);

        k_msleep(SENSOR_POLL_INTERVAL_MS);
    }
}

int8_t calculate_cursor_movement(const struct sensor_value *gyro, const struct sensor_value *accel, double scale)
{
    double g_val = (double)gyro->val1 + (double)gyro->val2 / 1000000.0;
    double a_val = (double)accel->val1 + (double)accel->val2 / 1000000.0;

    if (fabs(g_val) < CURSOR_DEADZONE) {
        return 0;
    }

    double accel_intensity = fabs(a_val);

    double dynamic_scale = scale + (accel_intensity * 5.0);
    if (dynamic_scale > scale * 3.0) dynamic_scale = scale * 3.0;

    double movement = g_val * dynamic_scale;

    if (movement > 127.0) {
        return 127;
    } else if (movement < -128.0){
        return -128;
    }

    return (int8_t)movement;
}

void send_hid_report()
{
    int err;
    const double scale = 100.0;
    struct sensor_data imu_get;

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
        k_msgq_get(&imu_msgq, &imu_get, K_FOREVER);
        int is_cursor = gpio_pin_get_dt(&cursor_enable_button);
        if (is_cursor != 0) {
            imu_get.gyro_data.y.val1 = 0;
            imu_get.gyro_data.y.val2 = 0;
            imu_get.gyro_data.z.val1 = 0;
            imu_get.gyro_data.z.val2 = 0;
        }

        send_mouse_report(
            -calculate_cursor_movement(&imu_get.gyro_data.z, &imu_get.accel_data.z, scale),
            -calculate_cursor_movement(&imu_get.gyro_data.y, &imu_get.accel_data.y, scale),0,0);
    }
}

K_THREAD_DEFINE(get_lsm6dsl_value_id, STACK_SIZE, get_lsm6dsl_value, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(send_hid_report_id, STACK_SIZE, send_hid_report, NULL, NULL, NULL, PRIORITY, 0, 0);
