# micro-ROS ESP32 SLAM 로버 코드 가이드

이 문서는 완성된 `micro_ros_esp32.c`의 소스 코드 구조와 각 부분의 기능을 설명합니다. 이 코드는 ESP32를 기반으로 ROS 2와 통신하며, IMU 데이터 전송 및 듀얼 모터 제어(디퍼런셜 드라이브)를 수행합니다.

## 1. 파일 구조 요약

- **`micro_ros_esp32.c`**: 메인 로직 파일 (센서 데이터 읽기, 모터 제어, ROS 2 통신).
- **`pinmap.h`**: 하드웨어 핀 번호 정의 파일.
- **`implementation_plan.md`**: 구현 단계별 계획서.

## 2. 코드 상세 설명

### A. 헤더 및 초기 설정
- **라이브러리**: `micro_ros_arduino.h`, `rclc`(ROS 2 Client Library for C), `Adafruit_BNO055` 등을 포함합니다.
- **핀 맵**: `#include "pinmap.h"`를 통해 하드웨어 의존성을 분리했습니다.
- **전역 변수**: 좌/우 인코더 틱(`volatile long`)과 ROS 2 메시지 객체들을 선언합니다.

### B. 인코더 및 인터럽트 (ISR)
- **`updateLeftEncoder()` / `updateRightEncoder()`**: 
  - 인코더 A상 신호의 Rising Edge 발생 시 호출됩니다.
  - B상의 상태를 확인하여 정방향(`++`) 또는 역방향(`--`)으로 틱을 계산합니다.
  - `IRAM_ATTR` 속성을 사용하여 인터럽트 응답 속도를 최적화했습니다.

### C. 모터 제어 로직
- **`setMotorSpeed()`**:
  - `ena`(PWM), `in1`, `in2` 핀을 제어하여 개별 모터의 방향과 속도를 결정합니다.
  - `-255 ~ 255` 사이의 값을 받아 음수는 역방향, 양수는 정방향으로 동작합니다.
- **`cmd_vel_callback()` (구독)**:
  - ROS 2의 `geometry_msgs/Twist` 메시지를 수신합니다.
  - **디퍼런셜 드라이브 수식** 적용:
    - `left_speed = linear.x - angular.z`
    - `right_speed = linear.x + angular.z`
  - 계산된 값을 255(PWM 최대치) 스케일로 변환하여 모터를 구동합니다.

### D. 센서 데이터 전송 (발행)
- **`timer_callback()`**:
  - 20Hz(50ms) 주기로 실행됩니다.
  - **인코더**: 현재 누적된 좌/우 틱 값을 발행합니다.
  - **IMU (BNO055)**: 쿼터니언(방향), 자이로스코프(각속도), 선형 가속도 데이터를 읽어 `sensor_msgs/Imu` 형식으로 발행합니다.

### E. 시스템 초기화 (`setup`)
1.  **통신**: `set_microros_transports()`로 시리얼 통신을 설정합니다.
2.  **I2C**: `Wire.begin(IMU_SDA, IMU_SCL)`로 IMU 통신 핀을 지정합니다.
3.  **GPIO**: `pinMode` 및 `attachInterrupt`를 통해 모터와 인코더 핀을 설정합니다.
4.  **micro-ROS**: Node, Publisher, Subscriber, Timer, Executor를 순서대로 초기화합니다.

## 3. ROS 2 인터페이스 (토픽)

| 토픽 명 | 메시지 타입 | 방향 | 설명 |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | **Sub** | 로봇 이동 속도 명령 |
| `/imu/data` | `sensor_msgs/msg/Imu` | **Pub** | IMU 센서 데이터 (방향/가속도) |
| `/encoder/left` | `std_msgs/msg/Int32` | **Pub** | 왼쪽 바퀴 인코더 틱 |
| `/encoder/right` | `std_msgs/msg/Int32` | **Pub** | 오른쪽 바퀴 인코더 틱 |

## 4. 하드웨어 주의 사항
- **GPIO 34, 35**: 입력 전용(Input Only) 핀이므로 모터 제어 출력용으로 사용할 수 없으며, 인코더 입력 시 외부 풀업 저항이 필요할 수 있습니다.
- **BNO055**: 전원이 투입된 후 약 1초 정도의 초기화 시간이 필요하므로 `delay(1000)`이 포함되어 있습니다.
