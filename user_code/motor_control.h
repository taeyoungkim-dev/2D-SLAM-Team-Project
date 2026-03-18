#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// 모터 핀 설정 구조체
typedef struct {
    int left_ena, left_in1, left_in2;
    int right_enb, right_in3, right_in4;
    int enc_left_a, enc_left_b;
    int enc_right_a, enc_right_b;
} motor_config_t;

// 모터 및 엔코더 초기화 (구조체로 핀 전달)
void motor_init(const motor_config_t *config);

// 모터 개별 목표 속도 설정 (ticks/sec)
void motor_set_speed(int left_target, int right_target);

// 엔코더 카운트 가져오기
long motor_get_left_encoder(void);
long motor_get_right_encoder(void);

// 엔코더 카운트 리셋
void motor_reset_encoders(void);

#endif /* MOTOR_CONTROL_H */
