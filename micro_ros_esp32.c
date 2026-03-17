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

// 핀 맵 헤더 포함
#include "pinmap.h"

// --- 전역 변수 ---
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// --- micro-ROS 객체 ---
rcl_publisher_t imu_publisher;
rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;
rcl_subscription_t cmd_vel_subscriber;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
geometry_msgs__msg__Twist twist_msg;

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

// --- 인터럽트 서비스 루틴 (인코더) ---
void IRAM_ATTR updateLeftEncoder() {
  if (digitalRead(ENCODER_LEFT_B) > 0) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void IRAM_ATTR updateRightEncoder() {
  if (digitalRead(ENCODER_RIGHT_B) > 0) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
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
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;   // 전진/후진 (m/s)
  float angular_z = msg->angular.z; // 회전 (rad/s)
  
  // 간단한 디퍼런셜 믹싱 (로봇의 실제 물리 파라미터에 따라 255.0 스케일 조정 필요)
  float l_speed = (linear_x - angular_z) * 255.0;
  float r_speed = (linear_x + angular_z) * 255.0;
  
  setMotorSpeed(MOTOR_LEFT_ENA, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, (int)l_speed);
  setMotorSpeed(MOTOR_RIGHT_ENB, MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, (int)r_speed);
}

// --- 콜백 2: 센서 데이터 발행 (Timer) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 1. 인코더 데이터 퍼블리시
    left_encoder_msg.data = left_encoder_ticks;
    right_encoder_msg.data = right_encoder_ticks;
    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));

    // 2. BNO055 IMU 데이터 퍼블리시
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
  // 주의: 34, 35번 핀은 ESP32에서 Input Only이며 내부 풀업이 없으므로 회로상 풀업 저항 필요
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

  allocator = rcl_get_default_allocator();

  // micro-ROS 초기화
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_slam_rover", "", &support));

  // 퍼블리셔 생성
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  RCCHECK(rclc_publisher_init_default(
    &left_encoder_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/left"));
  RCCHECK(rclc_publisher_init_default(
    &right_encoder_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/right"));

  // 서브스크라이버 생성
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // 타이머 생성 (20Hz)
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Executor 설정 (타이머 1개 + 서브스크라이버 1개 = 2개 핸들)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  
  // 메시지 헤더 설정
  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
