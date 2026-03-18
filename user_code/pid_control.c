#include "pid_control.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error_sum = 0.0f;
    pid->last_error = 0.0f;
    pid->output_limit_min = out_min;
    pid->output_limit_max = out_max;
    pid->integral_limit = (out_max > 0) ? out_max : 100.0f; // 기본 적분 제한값
}

float pid_compute(pid_controller_t *pid, float target, float current, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = target - current;
    
    // P term
    float p_out = pid->kp * error;

    // I term (Integral Windup 방지 포함)
    pid->error_sum += error * dt;
    if (pid->error_sum > pid->integral_limit) pid->error_sum = pid->integral_limit;
    if (pid->error_sum < -pid->integral_limit) pid->error_sum = -pid->integral_limit;
    float i_out = pid->ki * pid->error_sum;

    // D term
    float d_out = pid->kd * (error - pid->last_error) / dt;
    pid->last_error = error;

    float total_out = p_out + i_out + d_out;

    // 출력 제한 (Saturation)
    if (total_out > pid->output_limit_max) total_out = pid->output_limit_max;
    if (total_out < pid->output_limit_min) total_out = pid->output_limit_min;

    return total_out;
}

void pid_reset(pid_controller_t *pid) {
    pid->error_sum = 0.0f;
    pid->last_error = 0.0f;
}
