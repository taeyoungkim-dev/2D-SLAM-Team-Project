#include "odometry.h"
#include <math.h>

// 로봇 하드웨어 상수 (main_slam.c와 일치시켜야 함)
#define WHEEL_RADIUS        0.033   // 바퀴 반지름 (m)
#define WHEEL_BASE          0.160   // 바퀴 간 거리 (m)
#define ENCODER_TICKS_REV   1440    // 1회전당 엔코더 틱 수

static odometry_t current_odom;
static long last_left_ticks = 0;
static long last_right_ticks = 0;

void odom_init(void) {
    current_odom.x = 0.0f;
    current_odom.y = 0.0f;
    current_odom.theta = 0.0f;
    current_odom.v_linear = 0.0f;
    current_odom.v_angular = 0.0f;
    last_left_ticks = 0;
    last_right_ticks = 0;
}

void odom_update(long left_ticks, long right_ticks, float dt) {
    if (dt <= 0.0f) return;

    // 1. 엔코더 변화량 계산
    long d_left = left_ticks - last_left_ticks;
    long d_right = right_ticks - last_right_ticks;
    
    last_left_ticks = left_ticks;
    last_right_ticks = right_ticks;

    // 2. 각 바퀴의 이동 거리 (m) 계산
    float dist_per_tick = (2.0f * M_PI * WHEEL_RADIUS) / ENCODER_TICKS_REV;
    float d_left_m = (float)d_left * dist_per_tick;
    float d_right_m = (float)d_right * dist_per_tick;

    // 3. 로봇의 중심 이동 거리와 회전 각도 계산
    float d_center_m = (d_left_m + d_right_m) / 2.0f;
    float d_theta = (d_right_m - d_left_m) / WHEEL_BASE;

    // 4. 전역 좌표계 (x, y, theta) 업데이트
    current_odom.x += d_center_m * cosf(current_odom.theta + d_theta / 2.0f);
    current_odom.y += d_center_m * sinf(current_odom.theta + d_theta / 2.0f);
    current_odom.theta += d_theta;

    // 5. 속도 (v, w) 계산
    current_odom.v_linear = d_center_m / dt;
    current_odom.v_angular = d_theta / dt;

    // Theta 정규화 (-PI ~ PI)
    if (current_odom.theta > M_PI) current_odom.theta -= 2.0f * M_PI;
    if (current_odom.theta < -M_PI) current_odom.theta += 2.0f * M_PI;
}

odometry_t odom_get_data(void) {
    return current_odom;
}
