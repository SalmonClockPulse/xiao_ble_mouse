#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define IMU_PWR_PIN 8

static struct sensor_value  gyro_x_out, gyro_y_out;

static const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

int main(void)
{
    printf("Hello World!");
    return 0;
}
