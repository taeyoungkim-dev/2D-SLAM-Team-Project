# ESP32 SLAM 구동을 위한 기초 설계 계획

본 문서는 micro-ROS가 탑재된 ESP32 환경에서 SLAM(Simultaneous Localization and Mapping)을 구현하기 위한 기초 코드 설계 및 단계별 계획을 담고 있습니다.

## 1. 개요
- **목표**: ESP32를 통해 센서 데이터를 수집하고, micro-ROS를 통해 상위 노드(PC/Raspberry Pi)의 SLAM 패키지(예: Slam Toolbox, Cartographer)와 연동.
- **주요 구성**:
    - 하드웨어: 모터 드라이버, 엔코더, IMU 센서, LiDAR.
    - 소프트웨어: micro-ROS (ESP32), ROS2 (Host PC).

## 2. 주요 모듈 및 파일 구조 (user_code/)
- `motor_control.c/h`: 엔코더 피드백을 포함한 모터 제어 및 PID 제어.
- `odometry.c/h`: 엔코더 및 IMU 데이터를 활용한 오도메트리(Odometry) 계산.
- `lidar_interface.c/h`: LiDAR 데이터 수신 및 `sensor_msgs/msg/LaserScan` 메시지 변환.
- `main_slam.c`: micro-ROS 노드 생성 및 각 모듈 통합.

## 3. 구현 단계별 계획

### Phase 1: 기반 인프라 구축
- [ ] 모터 및 엔코더 드라이버 작성 (PWM 제어, 인터럽트 기반 엔코더 카운트).
- [ ] PID 제어를 통한 일정한 속도 유지 구현.

### Phase 2: 센서 데이터 수집 및 micro-ROS 연동
- [ ] IMU(MPU6050/9250) 드라이버 연동 및 가속도/자이로 데이터 추출.
- [ ] LiDAR 데이터 수신 및 파싱 (UART 통신).
- [ ] micro-ROS 퍼블리셔(Publisher) 설정:
    - `/odom` (nav_msgs/msg/Odometry)
    - `/scan` (sensor_msgs/msg/LaserScan)
    - `/tf` (tf2_msgs/msg/TFMessage)

### Phase 3: Odometry 계산 로직 최적화
- [ ] Differential Drive 기구학을 적용한 좌표 계산.
- [ ] IMU 데이터를 활용한 EKF(Extended Kalman Filter) 기반 오도메트리 보정 (PC 측에서 처리하거나 ESP32에서 단순 융합).

### Phase 4: 호스트 PC 연동 및 SLAM 실행
- [ ] micro-ROS Agent 실행.
- [ ] `slam_toolbox` 또는 `gmapping` 설정 파일 작성.
- [ ] Rviz2를 통한 지도 생성 및 로봇 경로 시각화 확인.

## 4. 고려 사항
- **메모리 관리**: ESP32의 SRAM 제약으로 인해 큰 크기의 LaserScan 배열 처리에 유의.
- **통신 대역폭**: UART(micro-ROS) 속도를 최대로 설정(예: 921600 bps)하여 데이터 지연 최소화.
- **전력 공급**: 모터 구동 시 발생하는 노이즈가 MCU 및 센서에 미치는 영향 방지.
