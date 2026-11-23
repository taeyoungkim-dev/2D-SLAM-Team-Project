"""Nav2 FollowPath 액션 실행: 구간 전송, 취소, 결과 처리, 실패/재시도 관리."""
from __future__ import annotations

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath

from grid_ops import inside
from line_safety import line_is_safe
from path_message import build_path_message
from parameters.thresholds import SHUTDOWN_CANCEL_WAIT_SEC
from tracking_diagnostics import log_tracking_diagnostic


class Nav2Executor:
    def __init__(self, ctx, action_client, active_pub):
        self.ctx = ctx
        self.action = action_client
        self.active_pub = active_pub

    def _path(self, world):
        s, p = self.ctx.state, self.ctx.params
        return build_path_message(world, s.meta, self.ctx.clock, p.path_resolution)

    def server_is_ready(self):
        return self.action.server_is_ready()

    def dispatch(self, pose):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        if s.action_busy or s.halted or not s.enabled:
            return
        leg = s.legs[0]
        _, goal = leg
        escape = leg in s.escape_legs  # 장애물 구역에서 빠져나오는 구간
        # Planned endpoints retain the extra clearance. The actual robot
        # may use the buffer while tracking back towards that path; applying
        # plan_safe to its measured pose would recreate the original stop.
        if escape:
            endpoint_mask = s.safe
        else:
            endpoint_mask = s.plan_safe if p.check_safe_position else s.free
        if (not inside(endpoint_mask, s.meta.pixel(*goal))
                or not line_is_safe(pose, goal, s.meta, ctx.area.execution_mask())
                or (not escape and ctx.area.leg_hits_zone(pose, goal))):
            s.legs.clear()
            s.pass_in_progress = False
            s.pass_start_count = None
            s.state = 'replanning'
            return
        if math.dist(pose, goal) < 1e-6:
            s.legs.popleft()
            return
        s.legs.popleft()
        s.active_leg, s.active_escape = (pose, goal), escape
        path = self._path([pose, goal])
        self.active_pub.publish(path)
        request = FollowPath.Goal()
        request.path, request.controller_id = path, p.controller_id
        request.goal_checker_id = s.goal_checker_id
        # Reserve before send_goal_async to prevent overlapping pending goals.
        s.goal_pending, s.cancel_sent, s.cancel_reason = True, False, ''
        s.action_since = time.monotonic()
        s.state = 'executing'
        try:
            self.action.send_goal_async(request).add_done_callback(self.goal_response)
        except Exception as exc:
            # A transport exception cannot prove that Nav2 did not accept it.
            ctx.block(f'FollowPath send state is unknown; refusing to send another goal: {exc}')

    def goal_response(self, future):
        ctx = self.ctx
        s = ctx.state
        s.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            s.goal_pending = True
            ctx.block(f'FollowPath acceptance is unknown; no replacement will be sent: {exc}')
            return
        if not handle.accepted:
            s.active_leg = None
            if s.cancel_reason:
                s.cancel_reason = ''
                s.legs.clear()
                s.pass_in_progress = False
                return
            self.failed('Nav2 rejected FollowPath')
            return
        s.goal_handle = handle
        try:
            handle.get_result_async().add_done_callback(self.goal_result)
        except Exception as exc:
            ctx.block(f'Cannot observe the active goal result: {exc}')
            return
        if s.cancel_reason or not s.enabled or s.halted:
            self.cancel(s.cancel_reason or 'paused')

    def cancel(self, reason):
        ctx = self.ctx
        s = ctx.state
        if not s.action_busy:
            return
        if not s.cancel_reason:
            detail = ''
            if reason == 'localization_jump' and s.tracker is not None:
                issue = s.tracker.last_diagnostic
                if issue is not None:
                    detail = f'; tracker_reason={issue.reason}; {issue.details}'
            elif reason == 'tf_lost' and s.last_tf_diagnostic:
                detail = '; ' + s.last_tf_diagnostic
            # One message per goal's first cancellation request, even if a
            # repeated diagnostic warning is currently throttled.
            ctx.logger.warning(
                f'FollowPath cancellation requested: reason={reason}; '
                f'goal_pending={s.goal_pending}; state={s.state}{detail}')
        s.cancel_reason = s.cancel_reason or reason
        if s.goal_handle is not None and not s.cancel_sent:
            s.cancel_sent = True
            try:
                s.goal_handle.cancel_goal_async().add_done_callback(self.cancel_response)
            except Exception as exc:
                ctx.block(f'Cancel transport failed; waiting for the original goal: {exc}')

    def cancel_response(self, future):
        ctx = self.ctx
        try:
            result = future.result()
            if not result.goals_canceling and ctx.state.action_busy:
                ctx.warn('cancel', 'Nav2 did not acknowledge cancellation; waiting for its terminal result')
        except Exception as exc:
            ctx.block(f'Cancellation response failed: {exc}')

    def goal_result(self, future):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        try:
            status = future.result().status
        except Exception as exc:
            ctx.block(f'Active goal terminal state is unknown; no replacement will be sent: {exc}')
            return
        leg, reason = s.active_leg, s.cancel_reason
        s.goal_handle, s.active_leg = None, None
        s.cancel_reason, s.cancel_sent = '', False
        self.active_pub.publish(self._path([]))
        if reason:
            s.legs.clear()
            s.pass_in_progress = False
            if reason == 'timeout':
                self.failed('FollowPath timed out')
            return
        if status == GoalStatus.STATUS_SUCCEEDED and leg is not None:
            measured = ctx.pose_provider.get()
            if p.check_endpoint:
                if measured is None:
                    self.failed(f'Endpoint check unavailable after Nav2 success: '
                                f'goal_xy=({leg[1][0]:.6f},{leg[1][1]:.6f})m; '
                                f'robot_xy=unavailable; error=unavailable; '
                                f'{s.last_tf_diagnostic}')
                    return
                pose, stamp_sec = measured
                error = math.dist(pose, leg[1])
                if error > p.endpoint_tolerance:
                    self.failed(
                        'Nav2 reported success but endpoint check failed: '
                        f'goal_xy=({leg[1][0]:.6f},{leg[1][1]:.6f})m; '
                        f'robot_xy=({pose[0]:.6f},{pose[1]:.6f})m; '
                        f'error={error:.6f}m > tolerance={p.endpoint_tolerance:.6f}m; '
                        f'tf_stamp={stamp_sec:.9f}s; frame={s.meta.frame}')
                    return
            if measured is not None:
                pose, stamp_sec = measured
                s.tracker.update(pose, stamp_sec, s.free, s.safe,
                                 p.tracker_max_speed, p.tracker_max_gap_sec,
                                 check_safe_position=p.check_safe_position)
                log_tracking_diagnostic(ctx, 'goal_result')
            if s.failures:
                ctx.logger.info(
                    f'FollowPath segment accepted: consecutive failures reset {s.failures} -> 0')
            s.failures = 0
            s.error = ''
            s.state = 'checking_coverage'
            return
        self.failed(f'FollowPath ended with status={status}; visited cells remain recorded')

    def failed(self, message):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        s.legs.clear()
        s.pass_in_progress = False
        s.pass_start_count = None
        s.failures += 1
        s.error = message
        if s.failures >= p.max_failures:
            ctx.block(message + f' Retry {s.failures}/{p.max_failures}. '
                      'Retry limit reached; fix the cause then call ~/resume.')
        else:
            s.state, s.retry_at = 'retry_wait', time.monotonic() + p.retry_delay_sec
            ctx.logger.warning(message + f' Retry {s.failures}/{p.max_failures}.')

    def wait_for_cancel(self, node):
        """종료 시 활성 goal 취소가 확인될 때까지 잠시 spin 한다."""
        s = self.ctx.state
        deadline = time.monotonic() + SHUTDOWN_CANCEL_WAIT_SEC
        while s.action_busy and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
        if s.action_busy:
            self.ctx.logger.error('Nav2 cancellation not confirmed before exit')
