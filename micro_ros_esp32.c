#include <micro_ros_arduino.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 메시지 타입
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>

// BNO055 IMU 센서 라이브러리
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <driver/gpio.h>

// --- 핀 맵 정의 ---
#define IMU_SDA 21
#define IMU_SCL 22

#define MOTOR_LEFT_ENA 33
#define MOTOR_LEFT_IN1 26
#define MOTOR_LEFT_IN2 27

#define MOTOR_RIGHT_IN3 14
#define MOTOR_RIGHT_IN4 13
#define MOTOR_RIGHT_ENB 32

#define ENCODER_LEFT_A 18
#define ENCODER_LEFT_B 19
#define ENCODER_RIGHT_A 16
#define ENCODER_RIGHT_B 17

// --- 전역 변수 ---
volatile int64_t left_encoder_ticks = 0;
volatile int64_t right_encoder_ticks = 0;

portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// 로봇 기구학 상수 (필요 시 실제 하드웨어 값으로 조정)
//바퀴 반지름
constexpr float WHEEL_RADIUS_M = 0.033f;
//두 바퀴 거리
constexpr float WHEEL_BASE_M = 0.160f;
//바퀴 최대 각속도
constexpr float MAX_WHEEL_ANGULAR_SPEED_RAD_S = 20.0f;
//바퀴 최대 선속도
constexpr float MAX_WHEEL_LINEAR_SPEED_MPS = WHEEL_RADIUS_M * MAX_WHEEL_ANGULAR_SPEED_RAD_S;
//cmd_vel 확인 시간
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500;
// 엔코더 1회전당 틱 수 (실제 하드웨어 스펙으로 교정 필요)
constexpr float ENCODER_TICKS_PER_REV = 960.0f;

// 속도 제어기(PID) 파라미터
constexpr float WHEEL_PID_KP = 18.0f;
constexpr float WHEEL_PID_KI = 4.0f;
constexpr float WHEEL_PID_KD = 0.2f;
constexpr float WHEEL_PID_I_CLAMP = 80.0f;
constexpr float TARGET_STOP_EPS_RAD_S = 0.05f;
constexpr int RIGHT_MOTOR_DIR_SIGN = -1;

#ifdef LED_BUILTIN
constexpr int STATUS_LED_PIN = LED_BUILTIN;
#else
constexpr int STATUS_LED_PIN = -1;
#endif

constexpr uint8_t IMU_CALIB_TARGET_LEVEL = 3;
constexpr uint32_t IMU_CALIB_CHECK_MS = 200;
constexpr uint32_t IMU_CALIB_TIMEOUT_MS = 60000;

uint32_t last_cmd_vel_ms = 0;
bool failsafe_stop_active = true;
bool emergency_stop_active = false;

float target_left_wheel_rad_s = 0.0f;
float target_right_wheel_rad_s = 0.0f;
float left_pid_integral = 0.0f;
float right_pid_integral = 0.0f;
float left_pid_prev_error = 0.0f;
float right_pid_prev_error = 0.0f;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// --- micro-ROS 객체 ---
rcl_publisher_t imu_publisher;
rcl_publisher_t joint_state_publisher;
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t emergency_stop_subscriber;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_state_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Bool emergency_stop_msg;

rosidl_runtime_c__String joint_names_storage[2];
double joint_position_storage[2] = {0.0, 0.0};
double joint_velocity_storage[2] = {0.0, 0.0};
double joint_effort_storage[2] = {0.0, 0.0};

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// 매크로 함수 (에러 체크용)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// 에러 발생 시 무한 루프
void error_loop(){
  while(1){
    delay(100);
  }
}

void setMotorSpeed(int ena, int in1, int in2, int speed);

void setStatusLed(bool on) {
  if (STATUS_LED_PIN >= 0) {
    digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
  }
}

void blinkStatusLedFast(uint8_t count) {
  if (STATUS_LED_PIN < 0) {
    return;
  }

  for (uint8_t i = 0; i < count; i++) {
    setStatusLed(true);
    delay(80);
    setStatusLed(false);
    delay(80);
  }
}

int32_t saturateToInt32(int64_t value) {
  if (value > INT32_MAX) {
    return INT32_MAX;
  }
  if (value < INT32_MIN) {
    return INT32_MIN;
  }
  return static_cast<int32_t>(value);
}

float clampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

void resetWheelPidState() {
  left_pid_integral = 0.0f;
  right_pid_integral = 0.0f;
  left_pid_prev_error = 0.0f;
  right_pid_prev_error = 0.0f;
}

void setTargetWheelSpeeds(float left_rad_s, float right_rad_s) {
  target_left_wheel_rad_s = clampFloat(left_rad_s, -MAX_WHEEL_ANGULAR_SPEED_RAD_S, MAX_WHEEL_ANGULAR_SPEED_RAD_S);
  target_right_wheel_rad_s = clampFloat(right_rad_s, -MAX_WHEEL_ANGULAR_SPEED_RAD_S, MAX_WHEEL_ANGULAR_SPEED_RAD_S);
}

int computeWheelPwmFromPid(float target_rad_s, float measured_rad_s, float * integral, float * prev_error, float dt_sec) {
  if (fabsf(target_rad_s) < TARGET_STOP_EPS_RAD_S && fabsf(measured_rad_s) < TARGET_STOP_EPS_RAD_S) {
    *integral = 0.0f;
    *prev_error = 0.0f;
    return 0;
  }

  float error = target_rad_s - measured_rad_s;
  if (dt_sec > 0.0f) {
    *integral += error * dt_sec;
  }
  *integral = clampFloat(*integral, -WHEEL_PID_I_CLAMP, WHEEL_PID_I_CLAMP);

  float derivative = 0.0f;
  if (dt_sec > 0.0f) {
    derivative = (error - *prev_error) / dt_sec;
  }
  *prev_error = error;

  float ff_pwm = (target_rad_s / MAX_WHEEL_ANGULAR_SPEED_RAD_S) * 255.0f;
  float pwm_f = ff_pwm + (WHEEL_PID_KP * error) + (WHEEL_PID_KI * (*integral)) + (WHEEL_PID_KD * derivative);
  pwm_f = clampFloat(pwm_f, -255.0f, 255.0f);

  return static_cast<int>(pwm_f);
}

void waitForImuCalibration() {
  uint8_t sys = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  bool led_state = false;
  bool timed_out = false;
  uint32_t start_ms = millis();

  while (true) {
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    bool calibrated = (sys >= IMU_CALIB_TARGET_LEVEL) && (gyro >= IMU_CALIB_TARGET_LEVEL) &&
                      (accel >= IMU_CALIB_TARGET_LEVEL) && (mag >= IMU_CALIB_TARGET_LEVEL);
    if (calibrated) {
      break;
    }

    if ((millis() - start_ms) > IMU_CALIB_TIMEOUT_MS) {
      timed_out = true;
      break;
    }

    led_state = !led_state;
    setStatusLed(led_state);
    delay(IMU_CALIB_CHECK_MS);
  }

  // 캘리브레이션 완료/타임아웃 후 LED 끔
  setStatusLed(false);

  if (timed_out) {
    blinkStatusLedFast(3);
  }
}

void setHeaderStamp(builtin_interfaces__msg__Time * stamp) {
  int64_t epoch_ms = static_cast<int64_t>(millis());
  stamp->sec = static_cast<int32_t>(epoch_ms / 1000);
  stamp->nanosec = static_cast<uint32_t>((epoch_ms % 1000) * 1000000ULL);
}

void initJointStateMessage() {
  joint_state_msg.name.data = joint_names_storage;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.capacity = 2;

  joint_names_storage[0].data = (char *)"left_wheel_joint";
  joint_names_storage[0].size = strlen(joint_names_storage[0].data);
  joint_names_storage[0].capacity = joint_names_storage[0].size + 1;

  joint_names_storage[1].data = (char *)"right_wheel_joint";
  joint_names_storage[1].size = strlen(joint_names_storage[1].data);
  joint_names_storage[1].capacity = joint_names_storage[1].size + 1;

  joint_state_msg.position.data = joint_position_storage;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.capacity = 2;

  joint_state_msg.velocity.data = joint_velocity_storage;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.capacity = 2;

  joint_state_msg.effort.data = joint_effort_storage;
  joint_state_msg.effort.size = 2;
  joint_state_msg.effort.capacity = 2;
}

void stopMotors() {
  setMotorSpeed(MOTOR_LEFT_ENA, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0);
  setMotorSpeed(MOTOR_RIGHT_ENB, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, 0);
}

void setImuCovariances() {
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.0;
    imu_msg.angular_velocity_covariance[i] = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }

  // 대각성분만 사용 (센서 특성에 맞춰 추후 보정 권장)
  imu_msg.orientation_covariance[0] = 0.02;
  imu_msg.orientation_covariance[4] = 0.02;
  imu_msg.orientation_covariance[8] = 0.02;

  imu_msg.angular_velocity_covariance[0] = 0.01;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[8] = 0.01;

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
}

// --- 인터럽트 서비스 루틴 (인코더) ---
void IRAM_ATTR updateLeftEncoder() {
  int b_level = gpio_get_level(static_cast<gpio_num_t>(ENCODER_LEFT_B));
  portENTER_CRITICAL_ISR(&encoder_mux);
  left_encoder_ticks += (b_level > 0) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&encoder_mux);
}

void IRAM_ATTR updateRightEncoder() {
  int b_level = gpio_get_level(static_cast<gpio_num_t>(ENCODER_RIGHT_B));
  portENTER_CRITICAL_ISR(&encoder_mux);
  // 오른쪽 모터 방향을 반전해 사용하므로 인코더 카운트 부호도 함께 반전
  right_encoder_ticks += (b_level > 0) ? -1 : 1;
  portEXIT_CRITICAL_ISR(&encoder_mux);
}

// --- 모터 제어 유틸리티 함수 ---
void setMotorSpeed(int ena, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(ena, abs(speed));
}

// --- 콜백 1: cmd_vel 수신 및 디퍼런셜 드라이브 제어 ---
void cmd_vel_callback(const void * msgin) {
  if (emergency_stop_active) {
    stopMotors();
    return;
  }

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;   // 전진/후진 (m/s)
  float angular_z = msg->angular.z; // 회전 (rad/s)

  // 표준 디퍼런셜 드라이브 모델
  float left_wheel_linear = linear_x - (angular_z * (WHEEL_BASE_M * 0.5f));
  float right_wheel_linear = linear_x + (angular_z * (WHEEL_BASE_M * 0.5f));

  float left_target_rad_s = 0.0f;
  float right_target_rad_s = 0.0f;
  if (WHEEL_RADIUS_M > 0.0f) {
    left_target_rad_s = left_wheel_linear / WHEEL_RADIUS_M;
    right_target_rad_s = right_wheel_linear / WHEEL_RADIUS_M;
  }
  setTargetWheelSpeeds(left_target_rad_s, right_target_rad_s);

  last_cmd_vel_ms = millis();
  failsafe_stop_active = false;
}

void emergency_stop_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  emergency_stop_active = msg->data;

  if (emergency_stop_active) {
    setTargetWheelSpeeds(0.0f, 0.0f);
    resetWheelPidState();
    stopMotors();
    failsafe_stop_active = true;
  }
}

// --- 콜백 2: 센서 데이터 발행 (Timer) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    static int64_t prev_left_ticks = 0;
    static int64_t prev_right_ticks = 0;
    static uint32_t prev_time_ms = 0;

    int64_t left_ticks_snapshot = 0;
    int64_t right_ticks_snapshot = 0;

    portENTER_CRITICAL(&encoder_mux);
    left_ticks_snapshot = left_encoder_ticks;
    right_ticks_snapshot = right_encoder_ticks;
    portEXIT_CRITICAL(&encoder_mux);

    // 1. JointState 퍼블리시
    uint32_t now_ms = millis();
    float dt_sec = 0.0f;
    if (prev_time_ms > 0 && now_ms > prev_time_ms) {
      dt_sec = (now_ms - prev_time_ms) / 1000.0f;
    }

    float left_pos_rad = (static_cast<float>(left_ticks_snapshot) / ENCODER_TICKS_PER_REV) * TWO_PI;
    float right_pos_rad = (static_cast<float>(right_ticks_snapshot) / ENCODER_TICKS_PER_REV) * TWO_PI;

    if (dt_sec > 0.0f) {
      float left_vel_rad_s = ((left_ticks_snapshot - prev_left_ticks) / ENCODER_TICKS_PER_REV) * TWO_PI / dt_sec;
      float right_vel_rad_s = ((right_ticks_snapshot - prev_right_ticks) / ENCODER_TICKS_PER_REV) * TWO_PI / dt_sec;

      // 1-1. 엔코더 피드백 기반 PID 속도 제어
      int left_pwm = computeWheelPwmFromPid(target_left_wheel_rad_s, left_vel_rad_s, &left_pid_integral, &left_pid_prev_error, dt_sec);
      int right_pwm = computeWheelPwmFromPid(target_right_wheel_rad_s, right_vel_rad_s, &right_pid_integral, &right_pid_prev_error, dt_sec);
      if (!emergency_stop_active) {
        setMotorSpeed(MOTOR_LEFT_ENA, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, left_pwm);
        setMotorSpeed(MOTOR_RIGHT_ENB, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, RIGHT_MOTOR_DIR_SIGN * right_pwm);
      }

      setHeaderStamp(&joint_state_msg.header.stamp);
      joint_position_storage[0] = left_pos_rad;
      joint_position_storage[1] = right_pos_rad;
      joint_velocity_storage[0] = left_vel_rad_s;
      joint_velocity_storage[1] = right_vel_rad_s;
      joint_effort_storage[0] = 0.0;
      joint_effort_storage[1] = 0.0;
      RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
    }

    prev_left_ticks = left_ticks_snapshot;
    prev_right_ticks = right_ticks_snapshot;
    prev_time_ms = now_ms;

    // 2. BNO055 IMU 데이터 퍼블리시
    setHeaderStamp(&imu_msg.header.stamp);

    // 쿼터니언 (방향)
    imu::Quaternion quat = bno.getQuat();
    imu_msg.orientation.w = quat.w();
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();

    // 자이로스코프 (각속도)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = gyro.x();
    imu_msg.angular_velocity.y = gyro.y();
    imu_msg.angular_velocity.z = gyro.z();

    // 선형 가속도
    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = linearaccel.x();
    imu_msg.linear_acceleration.y = linearaccel.y();
    imu_msg.linear_acceleration.z = linearaccel.z();

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void setup() {
  // micro-ROS 통신 설정
  set_microros_transports();

  if (STATUS_LED_PIN >= 0) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    setStatusLed(false);
  }
  
  // I2C 핀 설정 및 초기화
  Wire.begin(IMU_SDA, IMU_SCL);
  
  // 모터 핀 초기화
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  
  // 인코더 핀 초기화
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  // 인코더 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateRightEncoder, RISING);

  // BNO055 IMU 초기화
  if (!bno.begin()) {
    while (1) { delay(10); }
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  waitForImuCalibration();

  allocator = rcl_get_default_allocator();

  // micro-ROS 초기화
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_slam_rover", "", &support));

  // 퍼블리셔 생성
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));

  // 서브스크라이버 생성
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(
    &emergency_stop_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "emergency_stop"));

  // 타이머 생성 (20Hz)
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Executor 설정 (타이머 1개 + 서브스크라이버 2개 = 3개 핸들)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &emergency_stop_subscriber, &emergency_stop_msg, &emergency_stop_callback, ON_NEW_DATA));
  
  // 메시지 헤더 설정
  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  joint_state_msg.header.frame_id.data = (char *)"base_link";
  joint_state_msg.header.frame_id.size = strlen(joint_state_msg.header.frame_id.data);
  joint_state_msg.header.frame_id.capacity = joint_state_msg.header.frame_id.size + 1;

  setImuCovariances();
  initJointStateMessage();
  stopMotors();
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  uint32_t now_ms = millis();
  if ((last_cmd_vel_ms == 0 || (now_ms - last_cmd_vel_ms) > CMD_VEL_TIMEOUT_MS) && !failsafe_stop_active) {
    setTargetWheelSpeeds(0.0f, 0.0f);
    resetWheelPidState();
    stopMotors();
    failsafe_stop_active = true;
  }
}
