#TODO
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class TaeyoungExplorer(Node):
    def __init__(self):
        super().__init__('taeyoung_explorer_node')
        self.get_logger().info('Explorer Activated. Waiting for costmap...')
        
        # 1. Costmap 구독기 생성
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)
        
        self.map_data = None
        self.map_info = None

    def costmap_callback(self, msg):
        self.map_info = msg.info
        
        # 2. 1차원 튜플 데이터를 2D Numpy 배열로 변환 (핵심!)
        width = msg.info.width
        height = msg.info.height
        
        # ROS2의 OccupancyGrid는 1차원 배열이므로, 이를 (y, x) 형태의 2차원으로 재조립합니다.
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 3. 데이터가 잘 들어오는지 디버깅 출력
        free_space = np.count_nonzero(self.map_data == 0)
        unknown_space = np.count_nonzero(self.map_data == -1)
        lethal_space = np.count_nonzero(self.map_data == 254) # Nav2 기준 치명적 장애물
        
        self.get_logger().info(
            f'맵 수신 완료! [빈공간: {free_space}칸, 미지영역: {unknown_space}칸, 벽: {lethal_space}칸]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TaeyoungExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('탐색을 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()