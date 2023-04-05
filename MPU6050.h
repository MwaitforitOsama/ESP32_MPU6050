// mpu6050.h
#ifndef MPU6050_H
#define MPU6050_H
#include <math.h>

#include "driver/i2c.h"
#include "esp_log.h"
#define portTICK_RATE_MS 1

#define MPU6050_ADDRESS     0x68
#define MPU6050_ACCEL_REG   0x3B
#define MPU6050_GYRO_REG    0x43
#define M_PI           3.14159265358979323846  /* pi */
#define RAD_TO_DEG 57.2957795131
#define GYRO_SENSITIVITY 131.0 

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_accel_data_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_gyro_data_t;


typedef struct {
    double w;
    double x;
    double y;
    double z;
} mpu6050_quaternion_t;



#endif /* MPU6050_H */






static void i2c_master_init(i2c_port_t i2c_port)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(i2c_port, &conf);
    i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
}

void mpu6050_init(i2c_port_t i2c_port)
{
    i2c_master_init(i2c_port);
}

void mpu6050_read_accel_data(i2c_port_t i2c_port, mpu6050_accel_data_t* accel_data)
{
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("Error: Could not read MPU6050 accelerometer data.\n");
        return;
    }
    accel_data->x = (data[0] << 8) | data[1];
    accel_data->y = (data[2] << 8) | data[3];
    accel_data->z = (data[4] << 8) | data[5];
}


void mpu6050_read_gyro_data(i2c_port_t i2c_port, mpu6050_gyro_data_t* gyro_data)
{
uint8_t data[6];
i2c_cmd_handle_t cmd = i2c_cmd_link_create();
i2c_master_start(cmd);
i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | I2C_MASTER_WRITE, true);
i2c_master_write_byte(cmd, MPU6050_GYRO_REG, true);
i2c_master_start(cmd);
i2c_master_write_byte(cmd, MPU6050_ADDRESS << 1 | I2C_MASTER_READ, true);
i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
i2c_master_stop(cmd);
esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);
if (ret != ESP_OK) {
printf("Error: Could not read MPU6050 gyroscope data.\n");
return;
}
gyro_data->x = (data[0] << 8) | data[1];
gyro_data->y = (data[2] << 8) | data[3];
gyro_data->z = (data[4] << 8) | data[5];
}


double calculate_roll(mpu6050_accel_data_t* accel_data)
{
    double y = (double) accel_data->y / 16384.0;
    double z = (double) accel_data->z / 16384.0;

    // Calculate roll
    double roll = atan2(y, z) * RAD_TO_DEG;
    return roll;
}

double calculate_pitch(mpu6050_accel_data_t* accel_data)
{
    double x = (double) accel_data->x / 16384.0;
    double y = (double) accel_data->y / 16384.0;
    double z = (double) accel_data->z / 16384.0;

    // Calculate pitch
    double pitch = atan2(-x, sqrt(y*y + z*z)) * RAD_TO_DEG;
    return pitch;
}

float get_dt(uint32_t* last_time)
{
    uint32_t current_time = esp_log_timestamp();
    float dt = (current_time - *last_time) / 60.0; // Convert to seconds
    *last_time = current_time;
    return dt;
}


void calculate_rollgyro(mpu6050_gyro_data_t* gyro_data, float* roll, float* pitch, float* yaw, float dt)
{
    // Calculate roll and pitch angles
    *roll = atan2(gyro_data->y, gyro_data->z) * (180.0 / M_PI);
   

 
}

void calculate_pitchgyro(mpu6050_gyro_data_t* gyro_data, float* roll, float* pitch, float* yaw,float dt)
{
     *pitch = atan2(-gyro_data->x, sqrt(pow(gyro_data->y, 2) + pow(gyro_data->z, 2))) * (180.0 / M_PI);

}

void calculate_yawgyro(mpu6050_gyro_data_t* gyro_data, float* roll, float* pitch, float* yaw,float dt)
{
        // Calculate yaw angle
    *yaw += gyro_data->z / GYRO_SENSITIVITY * dt;

}

void esp_quaternion_normalize(float *qw, float *qx, float *qy, float *qz)
{
    // Calculate the magnitude of the quaternion
    float mag = sqrt((*qw * *qw) + (*qx * *qx) + (*qy * *qy) + (*qz * *qz));

    // Normalize the quaternion
    *qw /= mag;
    *qx /= mag;
    *qy /= mag;
    *qz /= mag;
}

void calculate_angles_from_quaternion(mpu6050_quaternion_t* quat, double* roll, double* pitch, double* yaw)
{
    // Roll angle (x-axis rotation)
    double sinr_cosp = 2 * (quat->w * quat->x + quat->y * quat->z);
    double cosr_cosp = 1 - 2 * (quat->x * quat->x + quat->y * quat->y);
    *roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch angle (y-axis rotation)
    double sinp = 2 * (quat->w * quat->y - quat->z * quat->x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp) * RAD_TO_DEG;
    else
        *pitch = asin(sinp) * RAD_TO_DEG;

    // Yaw angle (z-axis rotation)
    double siny_cosp = 2 * (quat->w * quat->z + quat->x * quat->y);
    double cosy_cosp = 1 - 2 * (quat->y * quat->y + quat->z * quat->z);
    *yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}



