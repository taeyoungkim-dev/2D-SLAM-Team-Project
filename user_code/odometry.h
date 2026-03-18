#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct {
    float x;        // 위치 (m)
    float y;        // 위치 (m)
    float theta;    // 방향 (rad)
    float v_linear; // 선속도 (m/s)
    float v_angular;// 각속도 (rad/s)
} odometry_t;

// 오도메트리 초기화
void odom_init(void);

// 엔코더 변화량을 바탕으로 좌표 업데이트
void odom_update(long left_ticks, long right_ticks, float dt);

// 현재 오도메트리 데이터 가져오기
odometry_t odom_get_data(void);

#endif /* ODOMETRY_H */
