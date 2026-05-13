#TODO
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.signal import convolve2d
from scipy.ndimage import label
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker

DEBUG_MODE = False

class TaeyoungExplorer(Node):
    def __init__(self):
        super().__init__('taeyoung_explorer_node')
        self.get_logger().info('Explorer Activated. Waiting for costmap...')

        #target_pos 시각화 publisher
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)

        #TF buffer, listener
        #로봇의 현재 위치를 callback함수를 쓰지 않고 얻어올 수 있음
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 운전수와 통신할 액션 클라이언트 생성
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 로봇이 지금 이동 중인지 확인하는 자물쇠(Lock)
        #navigating 중에는 명령하면 안된다
        self.is_navigating = False
        
        # 1. Costmap 구독기 생성
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)
        
        self.map_data = None
        self.map_info = None
        self.robot_pos_pixel = None
        
        # 블랙리스트 수첩 (못 가는 픽셀 좌표들을 모아둘 리스트)
        self.blacklist_pixels = []
        
        # 방금 쏜 미사일의 픽셀 좌표를 기억할 변수
        self.current_target_pixel = None
    def costmap_callback(self, msg):
        self.map_info = msg.info
        # 1. 내 위치부터 파악 (TF 조회)
        self.robot_pos_pixel = self.get_robot_pos_pixel()
        # 위치를 못 찾았으면 아직 움직일 때가 아님. 다음 지도를 기다림.
        if self.robot_pos_pixel is None:
            return
        # 자물쇠 확인: 이미 이동 중이라면 명령을 하달하지 않고 무시함
        if self.is_navigating:
            return
        # 2. 1차원 튜플 데이터를 2D Numpy 배열로 변환
        width = msg.info.width
        height = msg.info.height
        
        # ROS2의 OccupancyGrid는 1차원 배열이므로, 이를 (y, x) 형태의 2차원으로 재조립합니다.
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 3. 데이터가 잘 들어오는지 디버깅 출력
        if DEBUG_MODE :
            free_space = np.count_nonzero(self.map_data == 0)
            unknown_space = np.count_nonzero(self.map_data == -1)
            lethal_space = np.count_nonzero(self.map_data == 100) # Nav2 기준 치명적 장애물
            
            self.get_logger().info(
                f'맵 수신 완료! [빈공간: {free_space}칸, 미지영역: {unknown_space}칸, 벽: {lethal_space}칸]'
            )
#-------------------------Main logic--------------------#
        if self.map_data is not None:
            # 1. 스캔
            frontier_pixels = self.edge_scanning(self.map_data)
    
            if len(frontier_pixels) > 0:
                # 2. 묶기
                frontier_clusters = self.pixel_clustering(frontier_pixels)
                
                # 3. 거르기
                valid_clusters = self.noise_filtering(frontier_clusters)
                
                # 4. 조준
                target_pixel = self.target_selecting(valid_clusters)
                
                # 5. 발사
                if target_pixel is not None:
                    self.ordering(target_pixel, self.map_info)
        #TODO : End explore

    def edge_scanning(self,map_data):
        #map_data는 2차원 int8 matrix 형태
        # 1. 맵 분리: '빈공간' 지도와 '미지영역' 지도를 따로 만듭니다.
        # (조건에 맞으면 True(1), 아니면 False(0)인 흑백 지도가 됩니다)
        free_map = (map_data == 0).astype(np.int8)
        unknown_map = (map_data == -1).astype(np.int8)

        # 2. 커널(Kernel) 생성: 상하좌우를 살피기 위한 십자가 돋보기
        # 중앙(현재 픽셀)을 기준으로 상, 하, 좌, 우에 미지영역이 있는지 훑습니다.
        #edit
        #잘 안되면 8방향도 해보기
        kernel = np.array([
            [0, 1, 0],
            [1, 0, 1],
            [0, 1, 0]
        ])

        # 3. 엣지 감지 (Convolution의 마법)
        # unknown_map을 이 커널로 훑습니다.
        # 만약 주변(상하좌우)에 미지영역이 1개라도 있다면 결괏값이 1 이상이 됩니다.
        surrounded_by_unknown = convolve2d(unknown_map, kernel, mode='same')
        
        # 4. 프론티어 조건 결합
        # 조건 1: 내 발밑은 빈 공간이어야 함 (free_map == 1)
        # 조건 2: 내 주변 상하좌우 중 최소 한 곳은 미지영역이어야 함 (surrounded_by_unknown > 0)
        frontier_mask = (free_map == 1) & (surrounded_by_unknown > 0)

        # 5. 마스크에서 좌표 추출
        # True로 표시된 픽셀들의 좌표를 뽑아냅니다.
        # 결과 형태: N x 2 배열 (각 행은 [y, x] 좌표)
        frontier_pixels = np.argwhere(frontier_mask)

        return frontier_pixels
    def pixel_clustering(self, frontier_pixels):
        """
        N x 2 형태의 프론티어 좌표들을 받아서, 
        인접해 있는 픽셀들끼리 묶은 덩어리(Cluster)들의 리스트로 반환합니다.
        """
        # 0. 예외 처리: 프론티어가 하나도 발견되지 않았다면 빈 리스트 반환 후 조기 종료
        if len(frontier_pixels) == 0:
            return []

        # 1. 빈 도화지 준비 (원래 지도의 크기만큼 0으로 채워진 맵 생성)
        height = self.map_info.height
        width = self.map_info.width
        frontier_mask = np.zeros((height, width), dtype=np.int8)

        # 2. 도화지에 프론티어 픽셀들 도장 찍기 (Numpy 배열 썰기)
        # frontier_pixels[:, 0]은 모든 y좌표, [:, 1]은 모든 x좌표입니다.
        # for문 없이 한 방에 맵 위에 1(True)을 찍어버립니다.
        frontier_mask[frontier_pixels[:, 0], frontier_pixels[:, 1]] = 1

        # 3. 마법의 군집화 (Connected Component Labeling)
        # structure=np.ones((3,3))의 의미: 대각선으로만 붙어있어도 같은 덩어리로 인정하겠다 (8방향 연결)
        structure = np.ones((3, 3), dtype=int)
        
        # labeled_map: 1번 덩어리는 1, 2번 덩어리는 2로 채워진 지도
        # num_clusters: 총 몇 개의 덩어리가 나왔는지 개수
        labeled_map, num_clusters = label(frontier_mask, structure=structure)

        # 4. 덩어리별로 좌표를 모아서 리스트로 패키징
        frontier_clusters = []
        for i in range(1, num_clusters + 1):
            # 지도를 뒤져서 값이 'i'인 픽셀들의 좌표만 쏙 뽑아냅니다. (형태: N x 2 배열)
            cluster_coords = np.argwhere(labeled_map == i)
            frontier_clusters.append(cluster_coords)

        # 디버그 모드가 켜져있다면 현황 브리핑
        if DEBUG_MODE:
            self.get_logger().info(f'군집화 완료: 총 {num_clusters}개의 프론티어 덩어리 식별!')

        # Output 형태: [ array([[y,x], [y,x]...]), array([[y,x], ...]), ... ]
        return frontier_clusters

    def noise_filtering(self,frontier_clusters):
        """
        크기가 너무 작은 프론티어 덩어리(노이즈)를 가차 없이 버리고,
        기준치를 통과한 '진짜 목표' 덩어리들만 남깁니다.
        """
        # 나중에 __init__에서 self.min_frontier_size = 5 정도로 빼두시면 
        # 파라미터로 튜닝하기 좋습니다. 지금은 일단 5로 고정해보겠습니다.
        #edit
        min_size = 30
        
        # 파이썬의 꽃, 리스트 컴프리헨션(List Comprehension)을 사용한 1줄 컷!
        # "클러스터의 픽셀 수가 min_size 이상인 놈들만 살려서 새 리스트를 만든다"
        valid_clusters = [cluster for cluster in frontier_clusters if len(cluster) >= min_size]

        # 디버그 모드가 켜져있다면 생존자 브리핑
        # (주의: 아까 만든 DEBUG_MODE 변수가 전역 혹은 self 변수로 설정되어 있어야 합니다)
        if DEBUG_MODE:
            self.get_logger().info(
                f'필터링 완료: {len(frontier_clusters)}개 중 {len(valid_clusters)}개 cluster 생존'
            )

        return valid_clusters
    
    def target_selecting(self,valid_clusters):
        """
        살아남은 프론티어 덩어리들 중 로봇과 가장 가까운 덩어리의 
        '중심점(Centroid)'을 최종 타겟 픽셀로 반환합니다.
        
        :param valid_clusters: 노이즈 필터링을 통과한 덩어리 리스트
        :param robot_pos_pixel: 로봇의 현재 위치 [y, x] (Numpy array)
        """
        if len(valid_clusters) == 0:
            return None # 갈 곳이 없다면 탐색 종료 대기

        min_dist = float('inf') # 최소 거리를 저장할 변수 (초기값은 무한대)
        target_pixel = None # 최종 타겟 픽셀
        #edit
        blacklist_radius = 10.0
        for cluster in valid_clusters:
            #Medoid 맵핑
            # 1. 덩어리의 무게중심(Centroid) 계산
            # np.mean을 쓰면 y좌표들의 평균, x좌표들의 평균을 0.001초만에 구해줍니다.
            centroid_y = int(np.mean(cluster[:, 0]))
            centroid_x = int(np.mean(cluster[:, 1]))
            theoretical_centroid = np.array([centroid_y, centroid_x])
            distances_to_centroid = np.linalg.norm(cluster - theoretical_centroid, axis=1)
            best_pixel_idx = np.argmin(distances_to_centroid)
            centroid = cluster[best_pixel_idx]

            # ==========================================
            # [새로운 로직] 벽 관통 레이더망 (Numpy Slicing)
            # 타겟 주변을 11x11(반경 5픽셀) 크기로 썰어서 벽(100)을 스캔합니다.
            # 해상도 0.05m 기준, 사방으로 25cm를 검사하는 강력한 안전망입니다.
            # ==========================================
            R = 10
            y, x = centroid[0], centroid[1]
            
            # self.map_data의 크기를 가져와서 인덱스 초과(IndexError) 방지
            height, width = self.map_data.shape
            
            y_min = max(0, y - R)
            y_max = min(height, y + R + 1)
            x_min = max(0, x - R)
            x_max = min(width, x + R + 1)
            
            # 맵에서 타겟 주변 네모난 박스(ROI)만 싹둑 잘라옵니다.
            roi = self.map_data[y_min:y_max, x_min:x_max]
            
            # 박스 안에 치명적 장애물(100)이 단 하나라도 발견된다면?!
            if np.any(roi == 100):
                # "여긴 사지(死地)다! 혹은 가짜 방이다!" 하고 쿨하게 버립니다.
                self.blacklist_pixels.append(centroid)
                continue
            # ==========================================

            # [추가된 로직] 이 중심점이 블랙리스트 근처인지 검사
            is_blacklisted = False
            for bad_pixel in self.blacklist_pixels:
                # 블랙리스트 좌표와의 거리 계산
                dist_to_bad = np.linalg.norm(centroid - bad_pixel)
                if dist_to_bad < blacklist_radius:
                    is_blacklisted = True
                    break # 하나라도 걸리면 이 덩어리는 즉시 포기
            
            if is_blacklisted:
                continue # 이 덩어리는 스킵하고 다음 덩어리 검사!

            # [추가해야 할 핵심 방어 로직]
            # 로봇 위치와 타겟 후보의 픽셀 거리를 계산합니다.
            dist_to_robot = np.linalg.norm(centroid - self.robot_pos_pixel)
            
            # 해상도가 0.05m라면 6픽셀은 30cm입니다.
            # "내 발밑 30cm 이내에 있는 프론티어는, 어차피 지금 지워지고 있는 중인 '유령'이니까 무시해!"
            #edit
            if dist_to_robot < 10.0: 
                continue

            # 이 방어막을 통과한 놈들 중에서만 가장 가까운 타겟을 찾습니다.
            if dist_to_robot < min_dist:
                min_dist = dist_to_robot
                target_pixel = centroid

            # 2. 로봇 위치와의 거리 계산 (유클리디안 거리)
            # np.linalg.norm: 벡터의 길이(직선 거리)를 구해주는 강력한 함수입니다.
            dist = np.linalg.norm(centroid - self.robot_pos_pixel)

            # 3. 가장 가까운 타겟인지 확인 후 갱신
            if dist < min_dist:
                min_dist = dist
                target_pixel = centroid

        if DEBUG_MODE and target_pixel is not None:
            self.get_logger().info(f'타겟 록온! 픽셀 좌표: Y:{target_pixel[0]}, X:{target_pixel[1]} (거리: {min_dist:.2f}px)')

        # 최종 목표 픽셀 [y, x] 반환
        return target_pixel
    def ordering(self, target_pixel, map_info):
        """
        목표 픽셀을 현실의 미터(m) 좌표로 변환하고 Nav2로 쏘아 올립니다.
        """
        self.current_target_pixel = target_pixel

        res = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        
        pixel_y, pixel_x = target_pixel[0], target_pixel[1]

        target_x_m = (pixel_x * res) + origin_x
        target_y_m = (pixel_y * res) + origin_y

        # ==========================================
        # [추가된 코드] RViz2 시각화 마커 생성 및 퍼블리시
        # ==========================================
        marker = Marker()
        marker.header.frame_id = 'map' # 지도(map) 좌표계 기준
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_target'
        marker.id = 0                  # 아이디가 같으면 이전 마커를 지우고 그 자리에 덮어씁니다 (깔끔함)
        marker.type = Marker.SPHERE    # 둥근 구슬 모양
        marker.action = Marker.ADD

        # 위치 지정
        marker.pose.position.x = float(target_x_m)
        marker.pose.position.y = float(target_y_m)
        marker.pose.position.z = 0.1   # 바닥에 묻히지 않게 살짝(10cm) 띄워줍니다

        # 크기 (지름 0.2m)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # 색상 (빨간색, a는 투명도 1.0=불투명)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
        """
        목표 픽셀을 현실의 미터(m) 좌표로 변환하고 Nav2로 쏘아 올립니다.
        """
        # 발사 직전에 현재 타겟을 기억해둔다
        self.current_target_pixel = target_pixel

        # 1. 지도 정보 빼오기
        res = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        
        pixel_y, pixel_x = target_pixel[0], target_pixel[1]

        # 2. 픽셀 -> 현실 미터(m) 역변환 공식!
        # 현실 위치 = (픽셀 칸 수 * 해상도) + 지도 시작점
        target_x_m = (pixel_x * res) + origin_x
        target_y_m = (pixel_y * res) + origin_y

        # 3. Nav2에 보낼 미사일(Goal Message) 장전
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = target_x_m
        goal_msg.pose.pose.position.y = target_y_m
        
        # 탐색 중이므로 방향(Orientation)은 크게 중요하지 않음. 기본값(w=1.0)으로 정면 응시
        goal_msg.pose.pose.orientation.w = 1.0 

        if DEBUG_MODE:
            self.get_logger().info(f'출격 명령 하달! 목표 좌표: [X: {target_x_m:.2f}m, Y: {target_y_m:.2f}m]')

        # 4. 액션 서버(Nav2)가 준비되었는지 확인 후 발사!
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 서버가 응답하지 않습니다!')
            return

        # 발사 완료 및 자물쇠 잠금
        self.is_navigating = True
        
        # 비동기(Async) 발사: 결과가 나오면 self.goal_response_callback을 호출해라!
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def get_robot_pos_pixel(self):
        """
        TF를 조회하여 로봇의 현재 위치를 파악하고, 
        이를 Numpy 배열 인덱싱을 위한 [y, x] 픽셀 좌표로 변환합니다.
        """
        try:
            # 1. TF에 멱살 잡고 물어보기: "map 기준 base_link(로봇) 위치 내놔!"
            # rclpy.time.Time()은 '가장 최근의 쌩쌩한 데이터'를 달라는 뜻입니다.
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time()
            )
            
            # 2. 현실 좌표 획득 (단위: 미터 m)
            real_x = t.transform.translation.x
            real_y = t.transform.translation.y
            
            # 3. 지도 정보(원점, 해상도) 가져오기
            origin_x = self.map_info.origin.position.x
            origin_y = self.map_info.origin.position.y
            res = self.map_info.resolution
            
            # 4. 현실 좌표(m) -> 픽셀 좌표 변환 (핵심 공식!)
            # 픽셀 = (현실 위치 - 지도 시작점) / 해상도
            pixel_x = int((real_x - origin_x) / res)
            pixel_y = int((real_y - origin_y) / res)
            
            # 우리가 앞서 약속한 [y, x] 형태의 Numpy 배열로 반환
            return np.array([pixel_y, pixel_x])
            
        except TransformException as ex:
            # 로봇이 켜진 직후에는 TF가 아직 안 잡혀서 예외가 뜰 수 있습니다. 쿨하게 넘깁니다.
            if DEBUG_MODE:
                self.get_logger().warn(f'로봇 위치 찾는 중...: {ex}')
            return None
    def goal_response_callback(self, future):
        """명령이 Nav2 서버에 접수되었는지 확인"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            if DEBUG_MODE:
                self.get_logger().warn('Nav2가 목표를 거부했습니다. (블랙리스트 추가)')
            # 블랙리스트에 추가!
            if self.current_target_pixel is not None:
                self.blacklist_pixels.append(self.current_target_pixel)
            self.is_navigating = False
            return

        # 접수되었다면 주행이 끝날 때까지 기다렸다가 get_result_callback을 불러라
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """주행이 끝났을 때(성공/실패/취소) 호출됨"""
        result = future.result().result
        status = future.result().status
        
        # Status 코드 (4: 성공, 5: 취소, 6: 중단/실패)
        if status == 4:
            if DEBUG_MODE:
                self.get_logger().info('프론티어 도착 완료! 새로운 지형을 확보했습니다.')
        else:
            self.get_logger().warn(f'주행 실패(코드:{status}). (블랙리스트 추가)')
            # 가다가 막혀서 실패해도 블랙리스트에 추가!
            if self.current_target_pixel is not None:
                self.blacklist_pixels.append(self.current_target_pixel)

        # 성공했든 실패했든 상황이 끝났으므로 자물쇠를 푼다. (다음 costmap_callback에서 다시 탐색 시작)
        self.is_navigating = False
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