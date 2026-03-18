#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include "robot_hw.h"

// ROS2 토픽 핸들러 및 메시지
static rcl_subscription_t cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;
static rcl_publisher_t odom_pub, tf_pub, imu_pub, scan_pub;
static nav_msgs__msg__Odometry odom_msg;
static tf2_msgs__msg__TFMessage tf_msg;
static geometry_msgs__msg__TransformStamped tf_stamped;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__LaserScan scan_msg;
static float scan_ranges_data[360];

// 로봇 파라미터
#define WHEEL_BASE          0.160f
#define SCAN_SIZE           360

// 현재 로봇 좌표 (x, y, theta)
static float robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;

// [명령] /cmd_vel 수신 시 바퀴별 속도 계산 후 HW 명령 전달
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float v = msg->linear.x;
    float w = msg->angular.z;

    // Differential Drive Kinematics (m/s)
    float v_left = v - (w * WHEEL_BASE / 2.0f);
    float v_right = v + (w * WHEEL_BASE / 2.0f);

    hw_motors_set_velocity(v_left, v_right);
}

// [피드백] Odometry 업데이트 및 발행
void update_and_publish_odom(void) {
    static float last_l = 0, last_r = 0;
    float cur_l, cur_r;
    hw_encoders_get_state(&cur_l, &cur_r);

    float dl = cur_l - last_l;
    float dr = cur_r - last_r;
    last_l = cur_l; last_r = cur_r;

    float ds = (dl + dr) / 2.0f;
    float dtheta = (dr - dl) / WHEEL_BASE;

    robot_x += ds * cosf(robot_theta + dtheta / 2.0f);
    robot_y += ds * sinf(robot_theta + dtheta / 2.0f);
    robot_theta += dtheta;

    // Odom 메시지 구성 및 발행 (헤더, 좌표, 쿼터니언 등)
    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.orientation.z = sinf(robot_theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(robot_theta / 2.0f);
    rcl_publish(&odom_pub, &odom_msg, NULL);

    // TF 발행 (odom -> base_footprint)
    tf_stamped.transform.translation.x = robot_x;
    tf_stamped.transform.translation.y = robot_y;
    tf_stamped.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    tf_stamped.transform.rotation.w = odom_msg.pose.pose.orientation.w;
    rcl_publish(&tf_pub, &tf_msg, NULL);
}

// [센서] IMU 및 Scan 발행
void publish_sensors(void) {
    // IMU
    imu_data_t imu_raw;
    hw_imu_get_data(&imu_raw);
    imu_msg.linear_acceleration.x = imu_raw.ax;
    imu_msg.angular_velocity.z = imu_raw.gz;
    rcl_publish(&imu_pub, &imu_msg, NULL);

    // Scan
    hw_lidar_get_scan(scan_ranges_data);
    rcl_publish(&scan_pub, &scan_msg, NULL);
}

void slam_node_init(void) {
    hw_init(); // 하드웨어 초기화 (내부에서 핀 설정 수행)

    // micro-ROS 초기화, 노드 생성, Sub/Pub 등록 ... (생략 가능하나 구조 유지)
    // 100ms 주기로 update_and_publish_odom() 및 publish_sensors() 호출
}
