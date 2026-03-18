#include "motor_control.h"
#include "pid_control.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <esp_log.h>

static const char *TAG = "MOTOR_CONTROL";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          (20000)

// 제어용 전역 설정 변수
static motor_config_t g_cfg;

// PID 및 속도 변수
#define CONTROL_PERIOD_MS       20
#define KP                      2.0f
#define KI                      0.5f
#define KD                      0.1f

static volatile long left_encoder_count = 0;
static volatile long right_encoder_count = 0;
static long last_left_count = 0, last_right_count = 0;
static float target_left_speed = 0.0f, target_right_speed = 0.0f;
static float current_left_speed = 0.0f, current_right_speed = 0.0f;

static pid_controller_t pid_left, pid_right;
static esp_timer_handle_t pid_timer;

// 엔코더 ISR (인터럽트 발생 시 B상 상태 확인)
static void IRAM_ATTR left_encoder_isr(void* arg) {
    if (gpio_get_level(g_cfg.enc_left_a) == gpio_get_level(g_cfg.enc_left_b)) {
        left_encoder_count++;
    } else {
        left_encoder_count--;
    }
}

static void IRAM_ATTR right_encoder_isr(void* arg) {
    if (gpio_get_level(g_cfg.enc_right_a) == gpio_get_level(g_cfg.enc_right_b)) {
        right_encoder_count++;
    } else {
        right_encoder_count--;
    }
}

// 주기적 제어 루프
static void pid_timer_callback(void* arg) {
    float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    long current_left = left_encoder_count;
    long current_right = right_encoder_count;

    current_left_speed = (float)(current_left - last_left_count) / dt;
    current_right_speed = (float)(current_right - last_right_count) / dt;

    last_left_count = current_left;
    last_right_count = current_right;

    float pwm_left = pid_compute(&pid_left, target_left_speed, current_left_speed, dt);
    float pwm_right = pid_compute(&pid_right, target_right_speed, current_right_speed, dt);

    // 하드웨어 모터 출력 제어
    int pwm_l = (int)pwm_left;
    if (pwm_l > 0) {
        gpio_set_level(g_cfg.left_in1, 1); gpio_set_level(g_cfg.left_in2, 0);
    } else if (pwm_l < 0) {
        gpio_set_level(g_cfg.left_in1, 0); gpio_set_level(g_cfg.left_in2, 1);
        pwm_l = -pwm_l;
    } else {
        gpio_set_level(g_cfg.left_in1, 0); gpio_set_level(g_cfg.left_in2, 0);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, (pwm_l > 255) ? 255 : pwm_l);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);

    int pwm_r = (int)pwm_right;
    if (pwm_r > 0) {
        gpio_set_level(g_cfg.right_in3, 1); gpio_set_level(g_cfg.right_in4, 0);
    } else if (pwm_r < 0) {
        gpio_set_level(g_cfg.right_in3, 0); gpio_set_level(g_cfg.right_in4, 1);
        pwm_r = -pwm_r;
    } else {
        gpio_set_level(g_cfg.right_in3, 0); gpio_set_level(g_cfg.right_in4, 0);
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, (pwm_r > 255) ? 255 : pwm_r);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}

void motor_init(const motor_config_t *config) {
    g_cfg = *config; // 핀 설정 복사

    // GPIO 방향 설정
    gpio_set_direction(g_cfg.left_in1, GPIO_MODE_OUTPUT);
    gpio_set_direction(g_cfg.left_in2, GPIO_MODE_OUTPUT);
    gpio_set_direction(g_cfg.right_in3, GPIO_MODE_OUTPUT);
    gpio_set_direction(g_cfg.right_in4, GPIO_MODE_OUTPUT);

    // LEDC PWM 설정
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES, .freq_hz = LEDC_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_l = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER, .gpio_num = g_cfg.left_ena, .duty = 0, .hpoint = 0
    };
    ledc_channel_config(&ch_l);

    ledc_channel_config_t ch_r = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER, .gpio_num = g_cfg.right_enb, .duty = 0, .hpoint = 0
    };
    ledc_channel_config(&ch_r);

    // 엔코더 인터럽트 설정
    gpio_config_t enc_cfg = {
        .pin_bit_mask = (1ULL << g_cfg.enc_left_a) | (1ULL << g_cfg.enc_right_a),
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&enc_cfg);
    gpio_set_direction(g_cfg.enc_left_b, GPIO_MODE_INPUT);
    gpio_set_direction(g_cfg.enc_right_b, GPIO_MODE_INPUT);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(g_cfg.enc_left_a, left_encoder_isr, NULL);
    gpio_isr_handler_add(g_cfg.enc_right_a, right_encoder_isr, NULL);

    // PID 및 타이머 설정
    pid_init(&pid_left, KP, KI, KD, -255, 255);
    pid_init(&pid_right, KP, KI, KD, -255, 255);

    const esp_timer_create_args_t periodic_timer_args = { .callback = &pid_timer_callback, .name = "pid_timer" };
    esp_timer_create(&periodic_timer_args, &pid_timer);
    esp_timer_start_periodic(pid_timer, CONTROL_PERIOD_MS * 1000);

    ESP_LOGI(TAG, "Motor driver initialized with injected configuration.");
}

void motor_set_speed(int left_target, int right_target) {
    target_left_speed = (float)left_target;
    target_right_speed = (float)right_target;
    if (left_target == 0 && right_target == 0) { pid_reset(&pid_left); pid_reset(&pid_right); }
}

long motor_get_left_encoder(void) { return left_encoder_count; }
long motor_get_right_encoder(void) { return right_encoder_count; }
void motor_reset_encoders(void) { left_encoder_count = 0; right_encoder_count = 0; last_left_count = 0; last_right_count = 0; }
