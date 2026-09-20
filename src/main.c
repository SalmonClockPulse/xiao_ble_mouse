#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define STACK_SIZE 1024
#define PRIORITY 7

static struct sensor_value gyro_x_out, gyro_y_out;

static const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

void get_lsm6dsl_value()
{
    struct sensor_value odr_attr;

    if (!device_is_ready(lsm6dsl_dev)) {
        printf("sensor: device is not ready yet.");
        return;
    }

    odr_attr.val1 = 104;
    odr_attr.val2 = 0;

    if (sensor_attr_set(
        lsm6dsl_dev,
        SENSOR_CHAN_GYRO_XYZ,
        SENSOR_ATTR_SAMPLING_FREQUENCY,
        &odr_attr) < 0) {
            printf("Could not set sensor type and channel.");
            return;
        }
    while(1) {
        sensor_sample_fetch(lsm6dsl_dev);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &gyro_x_out);
        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &gyro_y_out);
        printk("gyro_x: {d}.{d} gyro_y: {d}.{d}", gyro_x_out.val1, gyro_x_out.val2, gyro_y_out.val1, gyro_y_out.val2);
    }
}

K_THREAD_DEFINE(get_lsm6dsl_value_id, STACK_SIZE, get_lsm6dsl_value, NULL, NULL, NULL, PRIORITY, 0, 0);
