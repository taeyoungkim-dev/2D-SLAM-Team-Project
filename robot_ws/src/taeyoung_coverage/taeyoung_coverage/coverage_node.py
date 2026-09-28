"""ROS2 노드: Cartographer의 /map을 받아 Coverage 경로를 만들고 Nav2에 전달.

ROS2 관련 작업(구독/TF/파라미터/Action 호출)만 여기서 담당한다. 실제 커버리지
알고리즘은 전부 :mod:`coverage_planner` / :mod:`map_utils` 에 있다
(README_coverage.md 9장 - 역할 분리).

데이터 흐름:
    /map (OccupancyGrid)
        -> map_utils.classify_map / compute_traversable_map
        -> coverage_planner.plan_coverage_path
        -> nav_msgs/Path 로 변환해 /coverage_path 로 publish
        -> nav2_msgs/action/FollowPath 로 Nav2 Controller Server에 전달

주의: 이 파일은 rclpy/nav2_msgs 등 ROS2 패키지가 설치되어 있어야 import할 수
있다. ROS2가 없는 환경(예: macOS 개발 PC)에서는 이 파일을 실행/테스트하지
않는다 - 순수 알고리즘 테스트는 test/test_map_utils.py, test/test_coverage_planner.py
를 사용한다 (README_coverage.md 14장).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from . import coverage_planner, map_utils
from .coverage_planner import CoveragePlanningError
from .coverage_tracker import CoverageTracker
from .map_utils import MapMeta


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    """평면(2D) yaw만 있는 회전을 quaternion (z, w)로 변환. x=y=0으로 가정."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class TaeyoungCoverage(Node):
    def __init__(self) -> None:
        super().__init__('taeyoung_coverage_node')

        # ------------------------------------------------------------------
        # 파라미터 (README_coverage.md 전반 - "ROS parameter로 조정 가능해야 함")
        # ------------------------------------------------------------------
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('robot_radius', map_utils.DEFAULT_ROBOT_RADIUS_M)
        self.declare_parameter('coverage_clearance', 0.02)
        self.declare_parameter('lane_spacing', 0.15)
        self.declare_parameter('coverage_width', 0.15)
        self.declare_parameter('min_interval_cells', 2)
        self.declare_parameter('min_region_cells', 20)
        self.declare_parameter('auto_plan', True)
        self.declare_parameter('send_to_nav2', True)
        self.declare_parameter('follow_path_action_name', 'follow_path')
        self.declare_parameter('tracker_update_period_sec', 0.5)

        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.coverage_clearance = float(self.get_parameter('coverage_clearance').value)
        self.lane_spacing = float(self.get_parameter('lane_spacing').value)
        self.coverage_width = float(self.get_parameter('coverage_width').value)
        self.min_interval_cells = int(self.get_parameter('min_interval_cells').value)
        self.min_region_cells = int(self.get_parameter('min_region_cells').value)
        self.auto_plan = bool(self.get_parameter('auto_plan').value)
        self.send_to_nav2 = bool(self.get_parameter('send_to_nav2').value)
        follow_path_action_name = str(self.get_parameter('follow_path_action_name').value)
        tracker_period = float(self.get_parameter('tracker_update_period_sec').value)

        self.get_logger().info(
            f'taeyoung_coverage 시작: robot_radius={self.robot_radius:.3f}m '
            f'(URDF 0.140x0.140 footprint 외접원 근사), '
            f'clearance={self.coverage_clearance:.3f}m, lane_spacing={self.lane_spacing:.3f}m'
        )

        # ------------------------------------------------------------------
        # /map 구독 (explorer_node.py와 동일한 QoS - Cartographer가 늦게 켜져도
        # 마지막 지도를 다시 받기 위해 TRANSIENT_LOCAL 사용)
        # ------------------------------------------------------------------
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        # ------------------------------------------------------------------
        # 결과/디버그 publisher
        # ------------------------------------------------------------------
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/coverage_path', path_qos)
        self.visited_pub = self.create_publisher(OccupancyGrid, '/coverage_visited_map', 1)
        self.remaining_pub = self.create_publisher(OccupancyGrid, '/coverage_remaining_map', 1)

        # ------------------------------------------------------------------
        # TF (explorer_node.py의 map->base_link 조회 패턴 재사용)
        # ------------------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # Nav2 FollowPath Action Client (explorer_node.py의 NavigateToPose
        # Action Client 콜백 패턴을 그대로 재사용 - goal_response/get_result)
        # ------------------------------------------------------------------
        self.follow_path_client = ActionClient(self, FollowPath, follow_path_action_name)
        self._active_goal_handle = None

        # ------------------------------------------------------------------
        # 내부 상태
        # ------------------------------------------------------------------
        self.meta: Optional[MapMeta] = None
        self.last_map_info = None  # 원본 OccupancyGrid.info (디버그 맵 재발행용)
        self.cleanable_map: Optional[np.ndarray] = None
        self.tracker: Optional[CoverageTracker] = None
        self.path_computed = False

        self.tracker_timer = self.create_timer(tracker_period, self.tracker_timer_callback)

    # ----------------------------------------------------------------------
    # /map 콜백
    # ----------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid) -> None:
        meta = MapMeta(
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            width=msg.info.width,
            height=msg.info.height,
        )
        self.meta = meta
        self.last_map_info = msg.info

        grid = map_utils.occupancy_grid_to_numpy(msg.info.width, msg.info.height, msg.data)
        classified = map_utils.classify_map(grid, self.occupied_threshold)
        self.cleanable_map = map_utils.compute_cleanable_map(classified)

        if self.tracker is None:
            self.tracker = CoverageTracker(meta, self.coverage_width)
        else:
            self.tracker.resize_if_needed(meta)

        if self.auto_plan and not self.path_computed:
            self.try_plan_and_send(classified, meta)

    # ----------------------------------------------------------------------
    # Coverage 계획 + Nav2 전달
    # ----------------------------------------------------------------------
    def try_plan_and_send(self, classified: np.ndarray, meta: MapMeta) -> None:
        traversable = map_utils.compute_traversable_map(
            classified, self.robot_radius, self.coverage_clearance, meta.resolution
        )

        robot_world = self.get_robot_world_pose(warn_if_missing=True)

        try:
            waypoints = coverage_planner.plan_coverage_path(
                traversable,
                meta,
                lane_spacing_m=self.lane_spacing,
                min_interval_cells=self.min_interval_cells,
                min_region_cells=self.min_region_cells,
                start_world=robot_world,
            )
        except CoveragePlanningError as exc:
            self.get_logger().error(f'Coverage 경로 검증 실패, 경로를 보내지 않음: {exc}')
            return

        if not waypoints:
            self.get_logger().warn('Coverage 대상 영역이 없습니다 (traversable 영역이 비어 있음).')
            return

        path_msg = self.build_path_msg(waypoints)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Coverage 경로 생성 완료: waypoint {len(waypoints)}개, /coverage_path publish')

        self.path_computed = True

        if self.send_to_nav2:
            self.send_follow_path(path_msg)

    def build_path_msg(self, waypoints) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(wp.x)
            pose.pose.position.y = float(wp.y)
            pose.pose.position.z = 0.0
            qz, qw = yaw_to_quaternion_zw(wp.yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        return path_msg

    # ----------------------------------------------------------------------
    # Nav2 FollowPath 연동 (NavigateToPose가 아니라 FollowPath를 사용하는 이유:
    # README_coverage.md 1, 5, 11장 - Global Planner가 경로를 다시 만들지 않고
    # 우리가 만든 경로를 그대로 추종하게 하기 위함)
    # ----------------------------------------------------------------------
    def send_follow_path(self, path_msg: Path) -> None:
        if self._active_goal_handle is not None:
            self.get_logger().warn('이미 진행 중인 FollowPath goal이 있어 새 요청을 보내지 않습니다.')
            return

        if not self.follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowPath 액션 서버(Nav2 Controller Server)가 응답하지 않습니다.')
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = ''  # 기본 controller 사용 (nav2 params의 default id)

        self.get_logger().info('Nav2 FollowPath 목표 전송...')
        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.follow_path_response_callback)

    def follow_path_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2가 FollowPath 목표를 거부했습니다.')
            return

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_result_callback)

    def follow_path_result_callback(self, future) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Coverage 주행 완료!')
        else:
            self.get_logger().warn(f'Coverage 주행 종료 (status 코드: {status}).')
        self._active_goal_handle = None

    # ----------------------------------------------------------------------
    # TF 조회 (explorer_node.py의 get_robot_pos_pixel()과 동일한 패턴,
    # 다만 여기서는 world (x,y) 그대로 반환한다 - pixel 변환은 호출부에서)
    # ----------------------------------------------------------------------
    def get_robot_world_pose(self, warn_if_missing: bool = False) -> Optional[tuple]:
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as ex:
            if warn_if_missing:
                self.get_logger().warn(f'로봇 위치(TF) 조회 실패, 시작점 없이 계획합니다: {ex}')
            return None

    # ----------------------------------------------------------------------
    # visited/remaining map 추적 (README_coverage.md 8장, 독립 모듈로 유지)
    # ----------------------------------------------------------------------
    def tracker_timer_callback(self) -> None:
        if self.tracker is None:
            return

        pose = self.get_robot_world_pose(warn_if_missing=False)
        if pose is not None:
            self.tracker.update_from_world_pose(pose[0], pose[1])

        if self.cleanable_map is not None and self.last_map_info is not None:
            self.publish_debug_maps()

    def publish_debug_maps(self) -> None:
        assert self.tracker is not None
        assert self.cleanable_map is not None

        remaining = self.tracker.remaining_map(self.cleanable_map)
        self.visited_pub.publish(self._numpy_bool_to_grid_msg(self.tracker.visited))
        self.remaining_pub.publish(self._numpy_bool_to_grid_msg(remaining))

    def _numpy_bool_to_grid_msg(self, mask: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self.last_map_info
        data = np.where(mask, 100, 0).astype(np.int8)
        msg.data = data.flatten().tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TaeyoungCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('taeyoung_coverage 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
