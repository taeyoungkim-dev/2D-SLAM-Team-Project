#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 메시지 타입
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>

// BNO055 IMU 센서 라이브러리
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- 핀 설정 (L298N 모터 드라이버 및 인코더 예시) ---
// 실제 결선에 맞게 핀 번호를 변경하세요.
#define ENA 14  // 모터 속도 제어 (PWM)
#define IN1 27  // 모터 방향 1
#define IN2 26  // 모터 방향 2
#define ENCA 34 // 인코더 A (인터럽트 핀)
#define ENCB 35 // 인코더 B

// --- 전역 변수 ---
volatile long encoder_ticks = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// --- micro-ROS 객체 ---
rcl_publisher_t imu_publisher;
rcl_publisher_t encoder_publisher;
rcl_subscription_t cmd_vel_subscriber;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int32 encoder_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// 매크로 함수 (에러 체크용)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// 에러 발생 시 무한 루프 (ESP32 내장 LED를 깜빡이게 추가하면 디버깅에 좋음)
void error_loop(){
  while(1){
    delay(100);
  }
}

// --- 인터럽트 서비스 루틴 (인코더) ---
void IRAM_ATTR updateEncoder() {
  if (digitalRead(ENCB) > 0) {
    encoder_ticks++;
  } else {
    encoder_ticks--;
  }
}

// --- 콜백 1: 라즈베리파이 -> ESP32 (cmd_vel 수신 및 모터 제어) ---
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x; // 전진/후진 속도
  
  // 간단한 스케일링 (실제 주행 시 모터 스펙에 맞춰 튜닝 필요)
  int pwm_val = map(abs(linear_x * 100), 0, 100, 0, 255); 
  if (pwm_val > 255) pwm_val = 255;
  
  if (linear_x > 0) { // 전진
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (linear_x < 0) { // 후진
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else { // 정지
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    pwm_val = 0;
  }
  analogWrite(ENA, pwm_val);
}

// --- 콜백 2: ESP32 -> 라즈베리파이 (센서 데이터 전송) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 1. 인코더 데이터 퍼블리시
    encoder_msg.data = encoder_ticks;
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));

    // 2. BNO055 IMU 데이터 퍼블리시
    // 쿼터니언 (방향)
    imu::Quaternion quat = bno.getQuat();
    imu_msg.orientation.w = quat.w();
    imu_msg.orientation.x = quat.x();
    imu_msg.orientation.y = quat.y();
    imu_msg.orientation.z = quat.z();

    // 자이로스코프 (각속도, rad/s)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu_msg.angular_velocity.x = gyro.x();
    imu_msg.angular_velocity.y = gyro.y();
    imu_msg.angular_velocity.z = gyro.z();

    // 선형 가속도 (중력 제외, m/s^2)
    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu_msg.linear_acceleration.x = linearaccel.x();
    imu_msg.linear_acceleration.y = linearaccel.y();
    imu_msg.linear_acceleration.z = linearaccel.z();

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

void setup() {
  // USB-C Serial을 통한 micro-ROS 통신 설정 (라즈베리파이와 연결)
  set_microros_transports();
  
  // 핀 초기화
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  
  // 인코더 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENCA), updateEncoder, RISING);

  // BNO055 IMU 초기화
  if (!bno.begin()) {
    // BNO055 연결 실패 시 무한 루프
    while (1) {
      delay(10);
    }
  }
  delay(1000);
  bno.setExtCrystalUse(true); // 외부 크리스탈 사용으로 정확도 향상

  allocator = rcl_get_default_allocator();

  // micro-ROS 초기화
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_rover_node", "", &support));

  // 퍼블리셔 생성 (IMU, Encoder)
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "rover/imu"));

  RCCHECK(rclc_publisher_init_default(
    &encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "rover/encoder_ticks"));

  // 서브스크라이버 생성 (cmd_vel)
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // 타이머 생성 (20Hz = 50ms 주기로 센서 데이터 전송)
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Executor 생성 (핸들 개수: 타이머 1개 + 서브스크라이버 1개 = 2)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  
  // IMU 메시지 frame_id 설정 (선택 사항이지만 tf 변환할 때 유용함)
  imu_msg.header.frame_id.data = (char * )"imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
}

void loop() {
  delay(10);
  // micro-ROS 이벤트 루프 실행
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}