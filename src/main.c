#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define STACK_SIZE 1024
#define PRIORITY 7

struct sensor_data {
    struct sensor_value x;
    struct sensor_value y;
    struct sensor_value z;
};

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

K_THREAD_DEFINE(get_lsm6dsl_value_id, STACK_SIZE, get_lsm6dsl_value, NULL, NULL, NULL, PRIORITY, 0, 0);
