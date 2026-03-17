# ESP32 micro-ROS SLAM 로버 구현 계획서

이 문서는 `pinmap.h`에 정의된 핀 배열을 바탕으로 `micro_ros_esp32.c`를 완성하여 IMU와 듀얼 인코더를 지원하는 가이드를 제공합니다.

## 1. 하드웨어 구성 요약 (pinmap.h 기준)

| 장치 | 기능 | 핀 번호 | 비고 |
| :--- | :--- | :--- | :--- |
| **IMU (BNO055)** | SDA / SCL | 21 / 22 | I2C 통신 |
| **좌측 모터** | ENA / IN1 / IN2 | 33 / 26 / 27 | PWM 속도 제어 및 방향 |
| **우측 모터** | ENB / IN3 / IN4 | 32 / 14 / 12 | PWM 속도 제어 및 방향 |
| **좌측 인코더** | Phase A / B | 34 / 35 | 입력 전용 (풀업 저항 권장) |
| **우측 인코더** | Phase A / B | 16 / 17 | 인터럽트 지원 |

## 2. 코드 수정 및 완성 전략

### A. 헤더 및 핀 설정 통합
- `micro_ros_esp32.c` 상단의 하드코딩된 핀 정의를 제거하고 `#include "pinmap.h"`를 사용합니다.
- I2C 초기화 시 `Wire.begin(IMU_SDA, IMU_SCL);`을 호출하여 명시적으로 핀을 지정합니다.

### B. 듀얼 인코더 지원
- 현재 단일 인코더 변수(`encoder_ticks`)를 `left_encoder_ticks`, `right_encoder_ticks`로 분리합니다.
- 좌/우 각각에 대한 ISR(Interrupt Service Routine)을 구현합니다.
- **주의:** 핀 34, 35는 입력 전용이므로 내부 풀업이 불가능할 수 있어 회로상 풀업 저항 확인이 필요합니다.

### C. 디퍼런셜 드라이브 (Differential Drive) 제어
- `cmd_vel_callback`에서 수신한 `linear.x`(전진)와 `angular.z`(회전) 값을 조합하여 좌/우 모터의 속도를 각각 계산합니다.
  - `left_speed = linear_x - angular_z`
  - `right_speed = linear_x + angular_z`

### D. ROS 2 토픽 구조 개선
- **IMU:** `sensor_msgs/msg/Imu` 타입을 사용하여 `/imu/data` 토픽으로 발행.
- **인코더:** 좌/우 데이터를 각각 발행하거나, `nav_msgs/msg/Odometry` 구성을 위한 기초 데이터로 전송. (현재는 `std_msgs/msg/Int32` 2개 사용 권장)

## 3. 상세 구현 단계

1.  **전역 변수 확장:** 좌/우 인코더 틱 및 모터 제어 변수 선언.
2.  **인터럽트 함수 추가:** `updateLeftEncoder()`, `updateRightEncoder()` 구현.
3.  **Setup 함수 수정:**
    - `pinMode` 설정 시 `pinmap.h` 정의값 사용.
    - `attachInterrupt`를 좌/우 모두 설정.
    - IMU 초기화 전 `Wire.begin` 호출.
4.  **Timer Callback 수정:** 좌/우 인코더 값을 모두 포함하도록 퍼블리시 로직 수정.
5.  **Motor Control 로직:** `analogWrite`와 `digitalWrite`를 사용하여 듀얼 모터 제어 로직 완성.

## 4. 향후 확장성
- 현재는 단순 틱 값만 전송하지만, 나중에 ESP32 내부에서 간단한 Odometry(주행 거리계) 계산을 수행하여 `nav_msgs/msg/Odometry`를 직접 발행하도록 업그레이드할 수 있습니다.
