#ifndef PID_CONTROL_H
#define PID_CONTROL_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float error_sum;
    float last_error;
    float output_limit_min;
    float output_limit_max;
    float integral_limit;
} pid_controller_t;

// PID 컨트롤러 초기화
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

// PID 계산 실행
float pid_compute(pid_controller_t *pid, float target, float current, float dt);

// 오차 합산 리셋 (정지 시 호출)
void pid_reset(pid_controller_t *pid);

#endif /* PID_CONTROL_H */
