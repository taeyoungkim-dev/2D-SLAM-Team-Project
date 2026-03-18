#include "motor_control.h"
#include "pid_control.h"
#include "pinmap.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <esp_log.h>

static const char *TAG = "MOTOR_CONTROL";

// PWM 설정 (LEDC 사용)
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // 0-255 범위
#define LEDC_FREQUENCY          (20000)          // 20kHz

// PID 파라미터 및 주기 (수정 필요: 실제 모터 성능에 따라 튜닝)
#define CONTROL_PERIOD_MS       20   // 50Hz (20ms)
#define KP                      2.0f
#define KI                      0.5f
#define KD                      0.1f

// 엔코더 카운트 및 속도 변수
static volatile long left_encoder_count = 0;
static volatile long right_encoder_count = 0;
static long last_left_count = 0;
static long last_right_count = 0;

static float target_left_speed = 0.0f;  // 목표 속도 (ticks/sec)
static float target_right_speed = 0.0f;
static float current_left_speed = 0.0f;
static float current_right_speed = 0.0f;

static pid_controller_t pid_left;
static pid_controller_t pid_right;
static esp_timer_handle_t pid_timer;

// 엔코더 ISR (인터럽트 서비스 루틴)
static void IRAM_ATTR left_encoder_isr(void* arg) {
    if (gpio_get_level(ENCODER_LEFT_A) == gpio_get_level(ENCODER_LEFT_B)) {
        left_encoder_count++;
    } else {
        left_encoder_count--;
    }
}

static void IRAM_ATTR right_encoder_isr(void* arg) {
    if (gpio_get_level(ENCODER_RIGHT_A) == gpio_get_level(ENCODER_RIGHT_B)) {
        right_encoder_count++;
    } else {
        right_encoder_count--;
    }
}

// PID 타이머 콜백 (주기적으로 속도 계산 및 PWM 업데이트)
static void pid_timer_callback(void* arg) {
    float dt = (float)CONTROL_PERIOD_MS / 1000.0f;

    // 현재 엔코더 값 복사 (원자성 확보를 위해 짧게 유지)
    long current_left = left_encoder_count;
    long current_right = right_encoder_count;

    // 속도 계산 (ticks/sec)
    current_left_speed = (float)(current_left - last_left_count) / dt;
    current_right_speed = (float)(current_right - last_right_count) / dt;

    last_left_count = current_left;
    last_right_count = current_right;

    // PID 계산 (출력은 -255 ~ 255 PWM 값)
    float pwm_left = pid_compute(&pid_left, target_left_speed, current_left_speed, dt);
    float pwm_right = pid_compute(&pid_right, target_right_speed, current_right_speed, dt);

    // 하드웨어 모터 제어 호출 (실제 적용)
    // 주의: 내부 motor_set_speed와 충돌을 피하기 위해 직접 ledc 호출 권장
    
    // 왼쪽 모터
    int pwm_l = (int)pwm_left;
    if (pwm_l > 0) {
        gpio_set_level(MOTOR_LEFT_IN1, 1);
        gpio_set_level(MOTOR_LEFT_IN2, 0);
    } else if (pwm_l < 0) {
        gpio_set_level(MOTOR_LEFT_IN1, 0);
        gpio_set_level(MOTOR_LEFT_IN2, 1);
        pwm_l = -pwm_l;
    } else {
        gpio_set_level(MOTOR_LEFT_IN1, 0);
        gpio_set_level(MOTOR_LEFT_IN2, 0);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, (pwm_l > 255) ? 255 : pwm_l);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

    // 오른쪽 모터
    int pwm_r = (int)pwm_right;
    if (pwm_r > 0) {
        gpio_set_level(MOTOR_RIGHT_IN3, 1);
        gpio_set_level(MOTOR_RIGHT_IN4, 0);
    } else if (pwm_r < 0) {
        gpio_set_level(MOTOR_RIGHT_IN3, 0);
        gpio_set_level(MOTOR_RIGHT_IN4, 1);
        pwm_r = -pwm_r;
    } else {
        gpio_set_level(MOTOR_RIGHT_IN3, 0);
        gpio_set_level(MOTOR_RIGHT_IN4, 0);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, (pwm_r > 255) ? 255 : pwm_r);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}

void motor_init(void) {
    // 1. GPIO 방향 설정 (IN1, IN2, IN3, IN4)
    gpio_reset_pin(MOTOR_LEFT_IN1);
    gpio_set_direction(MOTOR_LEFT_IN1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR_LEFT_IN2);
    gpio_set_direction(MOTOR_LEFT_IN2, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(MOTOR_RIGHT_IN3);
    gpio_set_direction(MOTOR_RIGHT_IN3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR_RIGHT_IN4);
    gpio_set_direction(MOTOR_RIGHT_IN4, GPIO_MODE_OUTPUT);

    // 2. LEDC (PWM) 타이머 설정
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 3. LEDC 채널 설정 (ENA, ENB)
    ledc_channel_config_t ledc_channel_left = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_LEFT_ENA,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_left);

    ledc_channel_config_t ledc_channel_right = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_RIGHT_ENB,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_right);

    // 4. 엔코더 인터럽트 설정
    gpio_config_t enc_cfg = {
        .pin_bit_mask = (1ULL << ENCODER_LEFT_A) | (1ULL << ENCODER_RIGHT_A),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&enc_cfg);

    gpio_set_direction(ENCODER_LEFT_B, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_RIGHT_B, GPIO_MODE_INPUT);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_LEFT_A, left_encoder_isr, NULL);
    gpio_isr_handler_add(ENCODER_RIGHT_A, right_encoder_isr, NULL);

    // 5. PID 및 타이머 설정
    pid_init(&pid_left, KP, KI, KD, -255, 255);
    pid_init(&pid_right, KP, KI, KD, -255, 255);

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &pid_timer_callback,
        .name = "pid_timer"
    };
    esp_timer_create(&periodic_timer_args, &pid_timer);
    esp_timer_start_periodic(pid_timer, CONTROL_PERIOD_MS * 1000); // us 단위

    ESP_LOGI(TAG, "Motor control with PID initialized.");
}

void motor_set_speed(int left_target, int right_target) {
    // 이제 target_speed는 ticks/sec 단위로 해석됩니다.
    // 사용자가 외부에서 주는 값을 적절히 스케일링하여 저장합니다.
    target_left_speed = (float)left_target;
    target_right_speed = (float)right_target;
    
    if (left_target == 0 && right_target == 0) {
        pid_reset(&pid_left);
        pid_reset(&pid_right);
    }
}

long motor_get_left_encoder(void) {
    return left_encoder_count;
}

long motor_get_right_encoder(void) {
    return right_encoder_count;
}

void motor_reset_encoders(void) {
    left_encoder_count = 0;
    right_encoder_count = 0;
    last_left_count = 0;
    last_right_count = 0;
}
