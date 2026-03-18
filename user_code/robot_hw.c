#include "robot_hw.h"
#include "pinmap.h"
#include "motor_control.h"
#include "imu_interface.h"
#include "lidar_interface.h"
#include <math.h>

// 로봇 물리 상수
#define WHEEL_RADIUS        0.033f  // m
#define ENCODER_TICKS_REV   1440.0f // ticks

void hw_init(void) {
    // 1. 모터 및 엔코더 초기화 (핀 정보 주입)
    motor_config_t m_cfg = {
        .left_ena = MOTOR_LEFT_ENA, .left_in1 = MOTOR_LEFT_IN1, .left_in2 = MOTOR_LEFT_IN2,
        .right_enb = MOTOR_RIGHT_ENB, .right_in3 = MOTOR_RIGHT_IN3, .right_in4 = MOTOR_RIGHT_IN4,
        .enc_left_a = ENCODER_LEFT_A, .enc_left_b = ENCODER_LEFT_B,
        .enc_right_a = ENCODER_RIGHT_A, .enc_right_b = ENCODER_RIGHT_B
    };
    motor_init(&m_cfg);

    // 2. IMU 및 LiDAR 초기화
    imu_init(IMU_SDA, IMU_SCL);
    lidar_init();
}

void hw_motors_set_velocity(float left_v, float right_v) {
    // m/s -> ticks/sec 변환
    float ticks_per_m = ENCODER_TICKS_REV / (2.0f * M_PI * WHEEL_RADIUS);
    int l_ticks = (int)(left_v * ticks_per_m);
    int r_ticks = (int)(right_v * ticks_per_m);

    // 실제 하드웨어 제어 함수 호출
    motor_set_speed(l_ticks, r_ticks);
}

void hw_encoders_get_state(float *left_dist, float *right_dist) {
    // ticks -> m 변환
    float m_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / ENCODER_TICKS_REV;
    *left_dist = (float)motor_get_left_encoder() * m_per_tick;
    *right_dist = (float)motor_get_right_encoder() * m_per_tick;
}

void hw_imu_get_data(imu_data_t *data) {
    imu_read_data(data);
}

void hw_lidar_get_scan(float *ranges) {
    lidar_data_t scan_raw;
    if (lidar_get_scan(&scan_raw)) {
        for (int i = 0; i < 360; i++) {
            ranges[i] = scan_raw.ranges[i];
        }
    }
}
