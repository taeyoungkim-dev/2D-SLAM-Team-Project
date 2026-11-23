"""모든 기능 모듈을 연결하는 ROS 2 노드 (orchestrator).

여기서는 ROS 입출력(구독/발행/서비스/액션/TF/타이머)을 만들고, 기능 모듈 객체를
Context 에 연결하기만 한다. 실제 동작은 각 모듈에 있다.

  파라미터        parameter_loader  <- parameters/ (feature_flags / thresholds / node_settings)
  지도 수신       map_updater       (map_input, traversability, coverage_tracker)
  로봇 위치       robot_pose
  계획 영역       planning_area
  계획/수락       plan_manager      (coverage_planner, path_search, leg_splitter, path_message)
  goal checker    goal_checker
  Nav2 실행       nav2_executor     (tracking_diagnostics)
  홈 복귀         home_return       (home_geometry, route_planner; 스위치 topic /return_home)
  장애물 구역     obstacle_zones    (스폰 launch 파일 위치 + SDF 반지름 -> 청소 불가 구역)
  완료 판정       pass_completion
  상태/디버그 발행 status_publisher
  서비스          service_handlers
  주기 흐름       tick_controller
"""
from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from cleaner_context import Context
from cleaner_state import CleanerState
from goal_checker import GoalCheckerConfigurator
from home_geometry import yaw_from_quaternion
from home_return import HomeReturn
from map_updater import MapUpdater
from nav2_executor import Nav2Executor
from obstacle_zones import ObstacleZoneInput
from parameter_loader import load_parameters
from parameters.node_settings import (COVERAGE_ACTIVE_PATH_TOPIC, COVERAGE_PATH_TOPIC,
                                      COVERAGE_STATUS_TOPIC, DEBUG_MAP_KEYS, NODE_NAME)
from parameters.thresholds import OBSTACLE_REFRESH_PERIOD_SEC
from pass_completion import PassCompletion
from plan_manager import PlanManager
from planning_area import PlanningArea
from robot_pose import RobotPoseProvider
from service_handlers import ServiceHandlers
from startup_log import log_startup
from status_publisher import StatusPublisher
from tick_controller import TickController


class CoverageCleaner(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.params = params = load_parameters(self)
        self.state = state = CleanerState(params)
        self.ctx = ctx = Context(params, state, self.get_logger(), self.get_clock())

        # --- ROS 입출력 ---
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)
        path_pub = self.create_publisher(Path, COVERAGE_PATH_TOPIC, retained)
        active_pub = self.create_publisher(Path, COVERAGE_ACTIVE_PATH_TOPIC, retained)
        debug_pubs = {key: self.create_publisher(OccupancyGrid, '/coverage_' + key + '_map', retained)
                      for key in DEBUG_MAP_KEYS}
        status_pub = self.create_publisher(String, COVERAGE_STATUS_TOPIC, retained)
        # 스위치는 volatile: 이전 실행에서 남은 True 가 새 노드를 바로 복귀시키지 않게 한다.
        switch_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        switch_pub = self.create_publisher(Bool, params.return_home_topic, switch_qos)
        navigate_client = ActionClient(self, NavigateToPose, params.navigate_action_name)
        spin_client = ActionClient(self, Spin, params.spin_action_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        action = ActionClient(self, FollowPath, params.follow_path_action_name)
        server = params.controller_server.rstrip('/')
        get_params = self.create_client(GetParameters, server + '/get_parameters')
        set_params = self.create_client(SetParameters, server + '/set_parameters')

        # --- 기능 모듈 연결 ---
        ctx.pose_provider = RobotPoseProvider(ctx, self.tf_buffer)
        ctx.obstacles = ObstacleZoneInput(ctx, self.lookup_transform)
        ctx.map_updater = MapUpdater(ctx)
        ctx.area = PlanningArea(ctx)
        ctx.planner = PlanManager(ctx, path_pub)
        ctx.goal_checker = GoalCheckerConfigurator(ctx, get_params, set_params)
        ctx.executor = Nav2Executor(ctx, action, active_pub)
        ctx.status = StatusPublisher(ctx, status_pub, debug_pubs)
        ctx.completion = PassCompletion(ctx)
        ctx.home = HomeReturn(ctx, navigate_client, spin_client, switch_pub, action)
        self.handlers = ServiceHandlers(ctx)
        self.ticker = TickController(ctx)

        # Retained static maps and volatile threshold publishers both work.
        # Duplicate deliveries are filtered by geometry + classified contents.
        self.map_sub = self.create_subscription(OccupancyGrid, params.map_topic,
                                                ctx.map_updater.on_map, retained)
        self.map_live_sub = self.create_subscription(OccupancyGrid, params.map_topic,
                                                     ctx.map_updater.on_map, live)
        self.return_home_sub = self.create_subscription(Bool, params.return_home_topic,
                                                        ctx.home.on_switch, switch_qos)
        self.home_pose_sub = self.create_subscription(PoseStamped, params.home_pose_topic,
                                                      ctx.home.on_home_pose, retained)
        # 장애물 구역: 스폰 launch 파일의 위치를 map 좌표로 바꿔 한 번만 만든다(이후 갱신 없음).
        self.obstacle_timer = self.create_timer(OBSTACLE_REFRESH_PERIOD_SEC, self.refresh_obstacles)
        self.pause_srv = self.create_service(Trigger, '~/pause', self.handlers.on_pause)
        self.resume_srv = self.create_service(Trigger, '~/resume', self.handlers.on_resume)
        self.reset_srv = self.create_service(Trigger, '~/reset', self.handlers.on_reset)
        self.timer = self.create_timer(params.tracker_update_period_sec, self.ticker.tick)
        log_startup(self.get_logger(), params)

    def refresh_obstacles(self):
        self.ctx.obstacles.refresh()
        if self.ctx.obstacles.done:             # 구역이 고정되면 더 이상 확인하지 않는다
            self.obstacle_timer.cancel()

    def lookup_transform(self, target_frame, source_frame):
        """최신 TF target <- source 를 (x, y, yaw) 로 돌려준다. 없으면 예외."""
        t = self.tf_buffer.lookup_transform(target_frame, source_frame, Time()).transform
        q = t.rotation
        return t.translation.x, t.translation.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def close(self):
        self.state.enabled = False
        self.ctx.home.abort('shutdown')
        self.ctx.executor.cancel('shutdown')
        self.ctx.executor.wait_for_cancel(self)
        self.ctx.planner.shutdown()
