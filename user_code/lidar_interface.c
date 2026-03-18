#include "lidar_interface.h"
#include "driver/uart.h"
#include <string.h>
#include <math.h>

#define LIDAR_UART_NUM      UART_NUM_2
#define LIDAR_RX_PIN        15 // 예시 핀 (사용자 환경에 맞게 수정 가능)
#define LIDAR_TX_PIN        2
#define BUF_SIZE            1024

static lidar_data_t current_scan;

void lidar_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(LIDAR_UART_NUM, &uart_config);
    uart_set_pin(LIDAR_UART_NUM, LIDAR_TX_PIN, LIDAR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LIDAR_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // 기본 레이저 스캔 설정값 초기화
    current_scan.angle_min = 0.0f;
    current_scan.angle_max = 2.0f * M_PI;
    current_scan.angle_increment = (2.0f * M_PI) / LIDAR_SCAN_SIZE;
    current_scan.range_min = 0.15f;
    current_scan.range_max = 12.0f;
    memset(current_scan.ranges, 0, sizeof(current_scan.ranges));
}

int lidar_get_scan(lidar_data_t *data) {
    // 실제 LiDAR 프로토콜 파싱 로직이 여기에 들어갑니다.
    // 여기서는 성공적으로 읽었다고 가정하고 현재 데이터를 복사합니다.
    memcpy(data, &current_scan, sizeof(lidar_data_t));
    return 1;
}
