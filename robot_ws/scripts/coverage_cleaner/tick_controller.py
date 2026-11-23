"""주기 tick: 각 기능 모듈을 정해진 순서로 호출하는 제어 흐름.

순서: 홈 복귀 모드 확인 -> 완료 래치 확인 -> 지도 확인 -> 로봇 위치(TF) -> tracker 갱신 -> 대상 셀 갱신
      -> 디버그 발행 -> 일시정지/차단 확인 -> 진행 중 goal 감시 -> 재시도 대기
      -> Nav2/goal checker 준비 -> 계획 결과 수거 -> 구간 전송 -> 완료 판정/재계획
"""
from __future__ import annotations

import time

from coverage_types import PlanningError, PlanningMarginError
from parameters.thresholds import CANCEL_GRACE_SEC, DEBUG_PUBLISH_PERIOD_SEC
from tracking_diagnostics import log_tracking_diagnostic


class TickController:
    def __init__(self, ctx):
        self.ctx = ctx

    def tick(self):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        # 홈 복귀 스위치가 켜지면 청소 로직은 건너뛰고 복귀만 수행한다.
        if s.home_mode:
            ctx.home.tick()
            ctx.status.publish_status()
            return
        # Latch completion until explicit resume/reset. TF loss or map
        # updates after stopping must not start an unsolicited cleanup pass.
        if s.single_pass_finished:
            ctx.status.publish_status()
            return
        wall = time.monotonic()
        if s.meta is None:
            ctx.status.publish_status()
            return
        measured = ctx.pose_provider.get()
        if measured is None:
            s.tracker.previous = None
            s.legs.clear()
            s.pass_in_progress = False
            s.plan_version = -1
            s.target = None
            ctx.executor.cancel('tf_lost')
            if not s.halted:
                s.state = 'waiting_for_tf'
            ctx.status.publish_status()
            return
        pose, stamp_sec = measured
        s.last_pose = pose
        ctx.home.capture(pose)
        if s.enabled and not s.halted and p.send_to_nav2:
            continuous = s.tracker.update(pose, stamp_sec, s.free, s.safe,
                                          p.tracker_max_speed, p.tracker_max_gap_sec,
                                          check_safe_position=p.check_safe_position)
            if not continuous:
                s.legs.clear()
                s.pass_in_progress = False
                s.plan_version = -1
                s.target = None
                ctx.executor.cancel('localization_jump')
                log_tracking_diagnostic(ctx, 'tick')
        try:
            if s.target is None:
                ctx.area.refresh_target(pose)
        except PlanningMarginError as exc:
            ctx.area.wait_for_planning_area(exc)
            return
        except PlanningError as exc:
            ctx.executor.cancel('unsafe_pose')
            s.legs.clear()
            s.pass_in_progress = False
            s.state = 'waiting_for_safe_pose'
            ctx.warn('unsafe_pose', str(exc))
            ctx.status.publish_status()
            return
        if wall - s.last_debug >= DEBUG_PUBLISH_PERIOD_SEC:
            ctx.status.publish_debug()
            s.last_debug = wall
        if not s.enabled or s.halted:
            s.state = 'blocked' if s.halted else 'paused'
            return
        if s.action_busy:
            if wall - s.action_since > p.goal_timeout_sec:
                ctx.executor.cancel('timeout')
                if wall - s.action_since > p.goal_timeout_sec + CANCEL_GRACE_SEC:
                    ctx.block('Nav2 did not terminate/cancel its goal; no replacement goal will be sent')
            return
        if wall < s.retry_at:
            return
        if p.send_to_nav2:
            if not ctx.executor.server_is_ready():
                s.state = 'waiting_for_nav2'
                ctx.warn('nav2', f'Waiting for {p.follow_path_action_name}; will retry automatically')
                return
            if not ctx.goal_checker.ensure():
                return
        if not ctx.planner.collect_result(pose):
            return
        if not p.send_to_nav2:
            if s.preview_version != s.map_version and s.plan_future is None:
                ctx.planner.start_plan(pose)
            return
        if s.legs:
            ctx.executor.dispatch(pose)
            return
        ctx.completion.evaluate(pose)
