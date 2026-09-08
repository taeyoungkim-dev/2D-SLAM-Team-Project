#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

class FrontierFreezer(Node):
    def __init__(self):
        super().__init__('frontier_freezer')
        # 원본 프론티어 구독
        self.sub = self.create_subscription(MarkerArray, '/explore/frontiers', self.callback, 10)
        # 박제된 프론티어 발행
        self.pub = self.create_publisher(MarkerArray, '/frozen_frontiers', 10)
        self.get_logger().info('❄️ 프론티어 냉동고 가동! RViz2에서 /frozen_frontiers 토픽을 추가하세요.')

    def callback(self, msg):
        if not msg.markers: return
        
        frozen_msg = MarkerArray()
        for m in msg.markers:
            # explore_lite가 '마커 삭제(DELETE)' 명령을 보낸 건 무시하고 그리기 명령만 가로챔
            if m.action in [2, 3]: 
                continue
            
            # 마커 이름표를 바꾸고 유통기한을 0(무한대)으로 조작
            m.ns = "frozen"
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            frozen_msg.markers.append(m)
        
        # 조작된 마커를 RViz2로 발사!
        if frozen_msg.markers:
            self.pub.publish(frozen_msg)
            self.get_logger().info(f'❄️ 찰나의 순간을 포착! {len(frozen_msg.markers)}개의 프론티어를 얼렸습니다!')

def main():
    rclpy.init()
    node = FrontierFreezer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()