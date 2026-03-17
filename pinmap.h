#ifndef PINMAP_H
#define PINMAP_H

// ==========================================
// [ IMU 센서 (I2C) ]
// 전원: VIN(3V3), GND(GND)
// ==========================================
#define IMU_SDA 21
#define IMU_SCL 22

// ==========================================
// [ 모터 드라이버 (변속기) ]
// ==========================================
// 왼쪽 모터
#define MOTOR_LEFT_ENA 33
#define MOTOR_LEFT_IN1 26
#define MOTOR_LEFT_IN2 27

// 오른쪽 모터
#define MOTOR_RIGHT_IN3 14
#define MOTOR_RIGHT_IN4 12
#define MOTOR_RIGHT_ENB 32

// ==========================================
// [ 모터 인코더 ]
// 공통 전원: 파랑색(3V3), 초록색(GND)
// ==========================================
// 왼쪽 인코더
#define ENCODER_LEFT_A 34  // 노랑색 (주의: 입력 전용 핀, 외부 풀업 필요할 수 있음)
#define ENCODER_LEFT_B 35  // 흰색 (주의: 입력 전용 핀, 외부 풀업 필요할 수 있음)

// 오른쪽 인코더
#define ENCODER_RIGHT_A 16
#define ENCODER_RIGHT_B 17

#endif /* PINMAP_H */