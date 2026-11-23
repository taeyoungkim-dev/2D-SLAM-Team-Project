"""노드 런타임 상태. 모든 컴포넌트가 공유하는 변경 가능 필드를 한곳에 모았다.

디버깅 시 이 객체 하나만 보면 현재 상태를 알 수 있다.
"""
from __future__ import annotations

from collections import deque


class CleanerState:
    def __init__(self, params):
        # --- 지도 ---
        self.meta = self.info = self.free = self.safe = self.target = self.tracker = None
        self.plan_safe = None
        self.map_version = 0
        # --- FSM / 제어 ---
        self.enabled = bool(params.auto_plan)
        self.halted = False
        self.state, self.error = 'waiting_for_map', ''
        # --- FollowPath 실행 ---
        self.legs = deque()
        self.active_leg = None
        self.goal_handle = None
        self.goal_pending = False
        self.cancel_reason = ''
        self.cancel_sent = False
        self.action_since = 0.0
        # --- 계획 ---
        self.plan_future = None
        self.plan_version = -1
        self.preview_version = -1
        self.retry_at = 0.0
        self.failures = self.no_progress_passes = 0
        self.pass_start_count = None
        self.pass_in_progress = False
        self.single_pass_finished = False
        # --- 홈 복귀 ---
        self.home = None            # (x, y, yaw) map frame. 첫 TF / /home_pose / 파라미터로 채워진다.
        self.home_from_param = False
        self.home_mode = False      # True 면 tick 은 청소 대신 홈 복귀만 수행한다.
        # --- 진단/기타 ---
        self.last_pose = None
        self.last_yaw = 0.0         # RobotPoseProvider.get() 이 성공할 때마다 갱신
        self.last_debug = 0.0
        self.last_warning = {}
        self.last_tf_diagnostic = ''
        # --- Nav2 goal checker 설정 ---
        self.param_busy = False
        self.param_ready = not params.configure_goal_checker
        self.param_since = 0.0
        self.goal_checker_id = params.goal_checker_id

    @property
    def action_busy(self):
        return self.goal_pending or self.goal_handle is not None
