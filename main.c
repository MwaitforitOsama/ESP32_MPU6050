#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

#define portTick_RATE_MS 1

void app_main(void)
{
    mpu6050_init(I2C_NUM_0);

    while (1) {
        mpu6050_accel_data_t accel_data;
        mpu6050_read_accel_data(I2C_NUM_0, &accel_data);
        printf("Accel X: %d, Y: %d, Z: %d\n", accel_data.x, accel_data.y, accel_data.z);
        mpu6050_gyro_data_t gyro_data;
        mpu6050_read_gyro_data(I2C_NUM_0, &gyro_data);
        calculate_roll(&accel_data);
        calculate_pitch(&accel_data);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
