#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <stdint.h>

typedef struct {
    float ax, ay, az; // 가속도 (m/s^2)
    float gx, gy, gz; // 각속도 (rad/s)
} imu_data_t;

// IMU 초기화 (I2C 설정 및 센서 기동)
int imu_init(void);

// 최신 IMU 데이터 읽기
int imu_read_data(imu_data_t *data);

#endif /* IMU_INTERFACE_H */
