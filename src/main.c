#include <stdint.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>

#include "ble_hid.c"

#define STACK_SIZE 1024
#define PRIORITY 7
#define SENSOR_POLL_INTERVAL_MS 10
#define CURSOR_DEADZONE 0.05

#define CHANGE_SCALE_BUTTON_PUSHED (1U << 0)
#define LEFT_BUTTON_PUSHED (1U << 0)
#define MIDDLE_BUTTON_PUSHED (1U << 1)
#define RIGHT_BUTTON_PUSHED (1U << 2)

static const struct gpio_dt_spec cursor_enable_button = GPIO_DT_SPEC_GET(DT_ALIAS(cursor_enable_button), gpios);
static const struct gpio_dt_spec change_scale_button = GPIO_DT_SPEC_GET(DT_ALIAS(change_scale_button), gpios);
static const struct gpio_dt_spec left_button = GPIO_DT_SPEC_GET(DT_ALIAS(left_button), gpios);
static const struct gpio_dt_spec middle_button = GPIO_DT_SPEC_GET(DT_ALIAS(middle_button), gpios);
static const struct gpio_dt_spec right_button = GPIO_DT_SPEC_GET(DT_ALIAS(right_button), gpios);
static const double scale[3] = {10, 50, 100};

static uint8_t scale_index = 0;
static uint8_t is_push;

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

void change_scale()
{
    if (is_push & CHANGE_SCALE_BUTTON_PUSHED) {
        return;
    }
    scale_index += 1;
    uint8_t scale_size = sizeof(scale) / sizeof(scale[0]);
    if (scale_index >= scale_size){
        scale_index = 0;
    }
    is_push |= CHANGE_SCALE_BUTTON_PUSHED;
}

int8_t calculate_cursor_movement(const struct sensor_value *gyro, const struct sensor_value *accel)
{
    double g_sign = 1.0;
    if (gyro->val1 < 0 || gyro->val2 < 0) g_sign = -1.0;

    double g_val = labs(gyro->val1) + labs(gyro->val2) / 1000000.0;
    double a_val = labs(accel->val1) + labs(accel->val2) / 1000000.0;

    if (g_val < CURSOR_DEADZONE) {
        return 0;
    }

    double dynamic_scale = scale[scale_index] + (a_val * 5.0);
    if (dynamic_scale > scale[scale_index] * 3.0) dynamic_scale = scale[scale_index] * 3.0;

    double movement = g_sign * g_val * dynamic_scale;

    if (movement > 127.0) {
        return 127;
    } else if (movement < -128.0){
        return -128;
    }

    return (int8_t)movement;
}

uint8_t mouse_button_read()
{
    static uint8_t mouse_button_state = 0;

    if (gpio_pin_get_dt(&left_button)){
        mouse_button_state |= LEFT_BUTTON_PUSHED;
    } else {
        mouse_button_state &= ~LEFT_BUTTON_PUSHED;
    }

    if (gpio_pin_get_dt(&middle_button)){
        mouse_button_state |= MIDDLE_BUTTON_PUSHED;
    } else {
        mouse_button_state &= ~MIDDLE_BUTTON_PUSHED;
    }

    if (gpio_pin_get_dt(&right_button)){
        mouse_button_state |= RIGHT_BUTTON_PUSHED;
    } else {
        mouse_button_state &= ~RIGHT_BUTTON_PUSHED;
    }

    return mouse_button_state;
}

void send_hid_report()
{
    int err;
    struct sensor_data imu_get;

    gpio_pin_configure_dt(&cursor_enable_button, GPIO_INPUT);
    gpio_pin_configure_dt(&change_scale_button, GPIO_INPUT);
    gpio_pin_configure_dt(&left_button, GPIO_INPUT);
    gpio_pin_configure_dt(&middle_button, GPIO_INPUT);
    gpio_pin_configure_dt(&right_button, GPIO_INPUT);

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
        if (gpio_pin_get_dt(&change_scale_button)) {
            change_scale();
        } else {
            is_push &= ~CHANGE_SCALE_BUTTON_PUSHED;
        }


        k_msgq_get(&imu_msgq, &imu_get, K_FOREVER);
        int is_cursor = gpio_pin_get_dt(&cursor_enable_button);
        if (is_cursor != 0) {
            imu_get.gyro_data.y.val1 = 0;
            imu_get.gyro_data.y.val2 = 0;
            imu_get.gyro_data.z.val1 = 0;
            imu_get.gyro_data.z.val2 = 0;
        }

        send_mouse_report(
            -calculate_cursor_movement(&imu_get.gyro_data.z, &imu_get.accel_data.z),
            -calculate_cursor_movement(&imu_get.gyro_data.y, &imu_get.accel_data.y),
            mouse_button_read(),0);
    }
}

K_THREAD_DEFINE(get_lsm6dsl_value_id, STACK_SIZE, get_lsm6dsl_value, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(send_hid_report_id, STACK_SIZE, send_hid_report, NULL, NULL, NULL, PRIORITY, 0, 0);
