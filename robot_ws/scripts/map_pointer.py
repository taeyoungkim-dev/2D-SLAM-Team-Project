import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

class MapSniper(Node):
    def __init__(self):
        super().__init__('map_sniper_node')
        
        # 1. 맵 데이터 수신 (Transient Local QoS 적용)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        
        # 2. RViz2의 'Publish Point' 클릭 좌표 수신
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        
        self.map_data = None
        self.map_info = None
        self.get_logger().info('🎯 맵 스나이퍼 가동 대기 중... RViz2에서 [Publish Point] 툴로 맵을 클릭하세요!')

    def map_callback(self, msg):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def click_callback(self, msg):
        if self.map_data is None:
            self.get_logger().warn('아직 맵 데이터를 받지 못했습니다.')
            return

        # RViz2에서 클릭한 현실 좌표 (미터)
        click_x = msg.point.x
        click_y = msg.point.y

        # 지도 메타데이터
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        # 현실 좌표(m) -> 배열 인덱스(픽셀) 변환
        pixel_x = int((click_x - origin_x) / res)
        pixel_y = int((click_y - origin_y) / res)

        # 배열 범위 초과 방지
        height, width = self.map_data.shape
        if 0 <= pixel_x < width and 0 <= pixel_y < height:
            # 🚨 해당 픽셀의 확률값 추출
            pixel_value = self.map_data[pixel_y, pixel_x]
            
            # 보기 편하게 터미널에 출력
            self.get_logger().info(
                f'👆 클릭 좌표: [X:{click_x:.2f}m, Y:{click_y:.2f}m] ➔ 맵 확률값: {pixel_value}'
            )
        else:
            self.get_logger().warn('클릭한 위치가 지도 배열 범위를 벗어났습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = MapSniper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()