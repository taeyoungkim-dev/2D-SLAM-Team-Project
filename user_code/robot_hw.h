#ifndef ROBOT_HW_H
#define ROBOT_HW_H

#include <stdint.h>

// IMU 데이터 구조체
typedef struct {
    float ax, ay, az; // 가속도 (m/s^2)
    float gx, gy, gz; // 각속도 (rad/s)
} imu_data_t;

// 1. 시스템 초기화 (내부적으로 핀 설정 및 드라이버 기동)
void hw_init(void);

// 2. 구동부 제어 (입력: 바퀴별 목표 선속도 m/s)
void hw_motors_set_velocity(float left_v, float right_v);

// 3. 오도메트리 피드백 (출력: 바퀴별 누적 이동 거리 m)
void hw_encoders_get_state(float *left_dist, float *right_dist);

// 4. 센서 피드백
void hw_imu_get_data(imu_data_t *data);
void hw_lidar_get_scan(float *ranges);

#endif /* ROBOT_HW_H */
