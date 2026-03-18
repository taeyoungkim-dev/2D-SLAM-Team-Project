# ESP32 SLAM 로봇: 인터페이스 중심 설계 명세서

본 아키텍처는 하드웨어 종속성(Pin, Peripheral)을 상위 ROS 로직으로부터 완전히 격리하여, 하드웨어가 변경되어도 상위 노드 코드를 수정할 필요가 없는 **인터페이스 기반 설계**를 지향합니다.

## 1. 계층 구조 (Layered Architecture)

### 1.1 ROS Node Layer (`main_slam.c`)
- **역할**: ROS2 토픽 관리, 기구학(Kinematics) 계산, 데이터 발행/구독.
- **특징**: `pinmap.h`를 포함하지 않음. 오직 `robot_hw.h` 인터페이스만 호출.
- **단위**: 모든 데이터는 SI 단위($m, rad, m/s, rad/s$)로 처리.

### 1.2 Hardware Interface Layer (`robot_hw.h/c`)
- **역할**: 구체적인 핀 제어, 센서 통신(I2C/UART), PID 제어 수행.
- **특징**: 상위 계층에 추상화된 함수 제공. 내부적으로 `pinmap.h` 및 하드웨어 드라이버 사용.
- **주요 인터페이스**:
    - `hw_motors_set_velocity(float left, float right)`: 목표 선속도 설정.
    - `hw_encoders_get_state(float *l_dist, float *r_dist)`: 누적 이동 거리 획득.
    - `hw_imu_get_data(imu_data_t *data)`: 가속도/각속도 획득.
    - `hw_lidar_get_scan(float *ranges)`: 레이저 스캔 데이터 획득.

---

## 2. ROS2 토픽 인터페이스

| 토픽 명칭 | 메시지 타입 | 하드웨어 인터페이스 연결 |
| :--- | :--- | :--- |
| `/cmd_vel` | `Twist` | `hw_motors_set_velocity()` 호출 |
| `/odom` | `Odometry` | `hw_encoders_get_state()` 데이터 기반 계산 |
| `/imu/data_raw` | `Imu` | `hw_imu_get_data()` 데이터 변환 |
| `/scan` | `LaserScan` | `hw_lidar_get_scan()` 데이터 매핑 |

---

## 3. 작동 데이터 흐름 (Data Flow)

1.  **명령(Downstream)**: `PC(/cmd_vel)` -> `main_slam(Kinematics)` -> `robot_hw(PID Control)` -> `Motors(PWM)`
2.  **피드백(Upstream)**: `Sensors(Raw)` -> `robot_hw(Unit Conversion)` -> `main_slam(Pose Estimation)` -> `PC(/odom, /scan, /imu)`
