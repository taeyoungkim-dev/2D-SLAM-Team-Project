#ifndef LIDAR_INTERFACE_H
#define LIDAR_INTERFACE_H

#include <stdint.h>

#define LIDAR_SCAN_SIZE 360 // 1도 단위 360개 데이터 가정

typedef struct {
    float ranges[LIDAR_SCAN_SIZE]; // 거리 데이터 (m)
    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
} lidar_data_t;

// LiDAR 초기화 (UART 설정 등)
void lidar_init(void);

// 최신 스캔 데이터 가져오기 (성공 시 1 반환)
int lidar_get_scan(lidar_data_t *data);

#endif /* LIDAR_INTERFACE_H */
