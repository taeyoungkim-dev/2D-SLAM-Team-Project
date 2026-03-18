#include "imu_interface.h"
#include "pinmap.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "IMU_INTERFACE";

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000
#define MPU6050_ADDR            0x68

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43

// Scales (for default ±2g and ±250deg/s)
#define ACCEL_SCALE             16384.0f
#define GYRO_SCALE              131.0f
#define GRAVITY                 9.80665f
#define DEG_TO_RAD              (M_PI / 180.0f)

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_SDA,
        .scl_io_num = IMU_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int imu_init(void) {
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C master init failed");
        return -1;
    }

    // Wake up MPU6050 (0 to sleep register)
    if (mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wakeup failed");
        return -1;
    }

    ESP_LOGI(TAG, "IMU MPU6050 initialized successfully.");
    return 0;
}

int imu_read_data(imu_data_t *data) {
    uint8_t buffer[14];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return -1;

    // Convert raw to scaled values
    int16_t raw_ax = (buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (buffer[2] << 8) | buffer[3];
    int16_t raw_az = (buffer[4] << 8) | buffer[5];
    int16_t raw_gx = (buffer[8] << 8) | buffer[9];
    int16_t raw_gy = (buffer[10] << 8) | buffer[11];
    int16_t raw_gz = (buffer[12] << 8) | buffer[13];

    // Accel (g -> m/s^2)
    data->ax = (float)raw_ax / ACCEL_SCALE * GRAVITY;
    data->ay = (float)raw_ay / ACCEL_SCALE * GRAVITY;
    data->az = (float)raw_az / ACCEL_SCALE * GRAVITY;

    // Gyro (deg/s -> rad/s)
    data->gx = (float)raw_gx / GYRO_SCALE * DEG_TO_RAD;
    data->gy = (float)raw_gy / GYRO_SCALE * DEG_TO_RAD;
    data->gz = (float)raw_gz / GYRO_SCALE * DEG_TO_RAD;

    return 0;
}
