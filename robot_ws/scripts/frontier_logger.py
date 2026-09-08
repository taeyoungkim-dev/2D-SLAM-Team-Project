#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from datetime import datetime

class FrontierLogger(Node):
    def __init__(self):
        super().__init__('frontier_logger')
        
        # /explore/frontiers 토픽 구독 (데이터 타입: MarkerArray)
        self.subscription = self.create_subscription(
            MarkerArray,
            '/explore/frontiers',
            self.listener_callback,
            10)
        
        # 로그 파일 생성 및 헤더 작성
        self.file_name = "frontier_log.txt"
        with open(self.file_name, 'a') as f:
            f.write(f"\n=========================================\n")
            f.write(f"--- 프론티어 감청 시작: {datetime.now()} ---\n")
            f.write(f"=========================================\n")
        
        self.get_logger().info('🕵️‍♂️ 프론티어 감시망 가동 완료! 데이터가 감지되면 터미널과 파일에 기록됩니다.')

    def listener_callback(self, msg):
        marker_count = len(msg.markers)
        
        # 마커가 1개라도 들어왔다면 기록
        if marker_count > 0:
            time_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            log_msg = f"[{time_str}]  프론티어 마커 {marker_count}개 감지됨!"
            
            # 1. 터미널에 즉시 출력
            self.get_logger().info(log_msg)
            
            # 2. 파일에 상세 내용 기록
            with open(self.file_name, 'a') as f:
                f.write(log_msg + '\n')
                
                # 첫 번째 프론티어의 대략적인 위치도 추적해서 기록
                try:
                    if msg.markers[0].points: # 선(Line) 형태일 경우
                        first_point = msg.markers[0].points[0]
                        f.write(f"    -> [위치 힌트] 첫 번째 포인트: x={first_point.x:.2f}, y={first_point.y:.2f}\n")
                    else: # 구(Sphere)나 큐브 형태일 경우
                        pos = msg.markers[0].pose.position
                        f.write(f"    -> [위치 힌트] 마커 중심 좌표: x={pos.x:.2f}, y={pos.y:.2f}\n")
                except Exception as e:
                    f.write(f"    -> 위치 정보 파싱 실패: {e}\n")

def main(args=None):
    rclpy.init(args=args)
    node = FrontierLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('감시를 종료합니다.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()