#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// 모터 방향 정의
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} motor_direction_t;

// 모터 및 엔코더 초기화
void motor_init(void);

// 모터 개별 속도 설정 (-255 ~ 255)
void motor_set_speed(int left_speed, int right_speed);

// 엔코더 카운트 가져오기
long motor_get_left_encoder(void);
long motor_get_right_encoder(void);

// 엔코더 카운트 리셋
void motor_reset_encoders(void);

#endif /* MOTOR_CONTROL_H */
