"""강제 홈 복귀(Return Home): 스위치 topic 이 켜지면 청소를 멈추고 시작 위치/yaw 로 돌아간다.

스위치: params.return_home_topic (std_msgs/Bool, 기본 /return_home)
  True  -> 청소 중지(FollowPath 취소) -> NavigateToPose(홈) -> yaw 가 오차 안에 들 때까지 Spin
  False -> 복귀 진행 중이면 중단 후 일시정지 상태 유지(재개는 ~/resume).
           복귀가 이미 끝났거나(done/failed) 시작 전이면 아무것도 바꾸지 않는다.
스위치는 "복귀 진행 중"을 뜻한다. 이 노드가 완료(done)/최종 실패(failed)/~/pause·~/resume 으로
인한 중단 시 스스로 False 로 내린다(서버 브릿지나 앱이 진행 여부를 topic 으로 볼 수 있다).
스위치를 켜는 주체(둘 다 같은 topic 을 쓴다)
  1. 이 노드: 청소가 끝났을 때(return_home_on_complete)
  2. 서버 브릿지(ros2_bridge_node.py): 앱의 중지/복귀 버튼 -> 서버 명령 return_to_dock

홈 위치(map frame, 우선순위): home_x/home_y/home_yaw 파라미터 > /home_pose topic > 첫 TF 위치.
복귀 단계(phase): stopping -> navigating -> verifying -> (aligning -> verifying)* -> done
  실패하면 retry_wait 후 재시도하며 max_failures 번째 실패에서 failed(state=return_failed).
완료 판정은 Nav2 결과가 아니라 실측 TF 로 한다(home_xy_tolerance, home_yaw_tolerance).
"""
from __future__ import annotations

import math
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import Bool

from home_geometry import at_home, home_error, yaw_from_quaternion
from parameters.thresholds import HOME_MAX_SPIN_TRIES

IDLE, STOPPING, NAVIGATING, ALIGNING, VERIFYING, RETRY_WAIT, DONE, FAILED = (
    'idle', 'stopping', 'navigating', 'aligning', 'verifying', 'retry_wait', 'done', 'failed')
ACTIVE = (STOPPING, NAVIGATING, ALIGNING, VERIFYING, RETRY_WAIT)
RETURN_STATES = ('returning_home', 'returned_home', 'return_failed')


class HomeReturn:
    def __init__(self, ctx, nav_client, spin_client, switch_pub):
        self.ctx = ctx
        self.nav, self.spin, self.switch_pub = nav_client, spin_client, switch_pub
        self.phase = IDLE
        self.handle = None          # 진행 중인 goal handle (NavigateToPose 또는 Spin)
        self.generation = 0         # 취소/재시작 전의 늦은 콜백을 걸러내는 번호
        self.attempts = self.spins = 0
        self.since = self.retry_at = 0.0
        p, s = ctx.params, ctx.state
        if not math.isnan(p.home_x):
            s.home, s.home_from_param = (p.home_x, p.home_y, p.home_yaw), True
            ctx.logger.info(f'Home pose from parameters: x={p.home_x:.3f} y={p.home_y:.3f} '
                            f'yaw={p.home_yaw:.3f}rad')

    # ------------------------------------------------------------ 홈 위치
    def capture(self, pose):
        """첫 유효 TF 를 홈으로 기록한다(파라미터/ /home_pose 로 이미 정해졌으면 유지)."""
        s = self.ctx.state
        if s.home is None:
            s.home = (pose[0], pose[1], s.last_yaw)
            self.ctx.logger.info(f'Home pose recorded from first TF: x={pose[0]:.3f} '
                                 f'y={pose[1]:.3f} yaw={s.last_yaw:.3f}rad')

    def on_home_pose(self, msg):
        ctx = self.ctx
        s = ctx.state
        if s.home_from_param:
            return
        if s.meta is not None and msg.header.frame_id and msg.header.frame_id != s.meta.frame:
            ctx.warn('home_frame', f'Ignoring /home_pose in frame {msg.header.frame_id!r}; '
                                   f'map frame is {s.meta.frame!r}')
            return
        q, t = msg.pose.orientation, msg.pose.position
        home = (t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if not all(math.isfinite(v) for v in home):
            return
        if s.home != home:
            s.home = home
            ctx.logger.info(f'Home pose set from topic: x={home[0]:.3f} y={home[1]:.3f} '
                            f'yaw={home[2]:.3f}rad')

    # ------------------------------------------------------------ 스위치
    def publish_switch(self, on):
        self.switch_pub.publish(Bool(data=bool(on)))

    def on_cleaning_complete(self):
        """청소 완료 시 호출. 내부 로직을 직접 부르지 않고 스위치 topic 을 켠다."""
        if self.ctx.params.return_home_on_complete:
            self.ctx.logger.info('Cleaning complete: turning the return-home switch on')
            self.publish_switch(True)

    def on_switch(self, msg):
        if msg.data:
            self.start()
        elif self.phase in ACTIVE:
            self.abort('switch_off')
        # done/failed/idle 에서의 False 는 스위치가 내려갔다는 알림일 뿐이므로 상태를 유지한다.

    def start(self):
        ctx = self.ctx
        s = ctx.state
        if self.phase in ACTIVE:
            return  # 이미 복귀 중
        ctx.logger.info('Return-home switch ON: stopping cleaning')
        ctx.pause('home_return')
        s.home_mode, s.error = True, ''
        self.generation += 1
        self.attempts = self.spins = 0
        if s.home is None:
            self.fail('Home pose is unknown (no TF received yet); cannot return home')
            return
        self.phase, s.state = STOPPING, 'returning_home'

    def abort(self, reason):
        """복귀를 중단한다. 청소는 일시정지 상태로 남는다."""
        s = self.ctx.state
        was_active = self.phase in ACTIVE
        if was_active:
            self.ctx.logger.info(f'Return home aborted: {reason}')
        self._cancel_goal()
        self.phase = IDLE
        s.home_mode = False
        if s.state in RETURN_STATES:
            s.state = 'paused'
        if was_active and reason not in ('switch_off', 'shutdown'):
            self.publish_switch(False)  # 외부에서 false 를 준 경우/종료 중에는 다시 내리지 않는다.

    # ------------------------------------------------------------ 주기 처리
    def tick(self):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        if self.phase not in ACTIVE:
            return
        if self.phase in (NAVIGATING, ALIGNING):
            limit = p.home_nav_timeout_sec if self.phase == NAVIGATING else p.home_spin_timeout_sec
            if time.monotonic() - self.since > limit:
                self._cancel_goal()
                self.attempt_failed(f'{self.phase} timed out after {limit:.0f}s')
            return
        if s.action_busy:  # 청소용 FollowPath 가 완전히 끝난 뒤에 홈 goal 을 보낸다.
            ctx.executor.cancel('home_return')
            ctx.warn('home_wait', 'Waiting for the FollowPath cancellation before returning home')
            return
        if self.phase == RETRY_WAIT and time.monotonic() < self.retry_at:
            return
        measured = ctx.pose_provider.get()
        if measured is None:
            ctx.warn('home_tf', 'Return home is waiting for a valid TF')
            return
        pose, yaw = measured[0], s.last_yaw
        if self.phase in (STOPPING, RETRY_WAIT):
            self.begin(pose, yaw)
        elif self.phase == VERIFYING:
            self.verify(pose, yaw)

    def begin(self, pose, yaw):
        ctx = self.ctx
        p, home = ctx.params, ctx.state.home
        distance, yaw_error = home_error(pose, yaw, home)
        if at_home(pose, yaw, home, p.home_xy_tolerance, p.home_yaw_tolerance):
            self.done(distance, yaw_error)
        elif distance <= p.home_xy_tolerance:
            self.send_spin(yaw_error)
        else:
            self.send_navigate()

    def verify(self, pose, yaw):
        ctx = self.ctx
        p, home = ctx.params, ctx.state.home
        distance, yaw_error = home_error(pose, yaw, home)
        if distance > p.home_xy_tolerance:
            self.attempt_failed(f'Position error {distance:.3f}m > home_xy_tolerance '
                                f'{p.home_xy_tolerance:.3f}m after the goal ended')
        elif abs(yaw_error) <= p.home_yaw_tolerance:
            self.done(distance, yaw_error)
        else:
            self.send_spin(yaw_error)

    # ------------------------------------------------------------ Nav2 goal
    def send_navigate(self):
        ctx = self.ctx
        s = ctx.state
        if not self.nav.server_is_ready():
            s.state = 'returning_home'
            ctx.warn('home_nav2', f'Waiting for {ctx.params.navigate_action_name}; will retry')
            return
        x, y, yaw = s.home
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = s.meta.frame
        goal.pose.header.stamp = ctx.clock.now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ctx.logger.info(f'Returning home: NavigateToPose x={x:.3f} y={y:.3f} yaw={yaw:.3f}rad')
        self._send(self.nav, goal, NAVIGATING, 'NavigateToPose')

    def send_spin(self, yaw_error):
        ctx = self.ctx
        if not self.spin.server_is_ready():
            ctx.warn('home_spin', f'Waiting for {ctx.params.spin_action_name}; will retry')
            return
        self.spins += 1
        if self.spins > HOME_MAX_SPIN_TRIES:
            self.fail(f'Yaw is still off by {math.degrees(yaw_error):.1f}deg after '
                      f'{HOME_MAX_SPIN_TRIES} Spin attempts')
            return
        timeout = ctx.params.home_spin_timeout_sec
        goal = Spin.Goal()
        goal.target_yaw = float(yaw_error)
        goal.time_allowance = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        ctx.logger.info(f'Aligning yaw: Spin {math.degrees(yaw_error):.1f}deg '
                        f'({self.spins}/{HOME_MAX_SPIN_TRIES})')
        self._send(self.spin, goal, ALIGNING, 'Spin')

    def _send(self, client, goal, phase, name):
        self.phase, self.since, self.handle = phase, time.monotonic(), None
        self.ctx.state.state = 'returning_home'
        generation = self.generation
        try:
            client.send_goal_async(goal).add_done_callback(
                lambda f: self._on_goal(f, generation, name))
        except Exception as exc:  # noqa: BLE001
            self.attempt_failed(f'{name} send failed: {exc}')

    def _on_goal(self, future, generation, name):
        stale = generation != self.generation
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            if not stale:
                self.attempt_failed(f'{name} goal response failed: {exc}')
            return
        if stale:  # 그 사이 취소/재시작됨: 늦게 수락된 goal 도 취소한다.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.attempt_failed(f'Nav2 rejected the {name} goal')
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, generation, name))

    def _on_result(self, future, generation, name):
        if generation != self.generation:
            return
        self.handle = None
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.attempt_failed(f'{name} result failed: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.phase = VERIFYING  # 성공 여부는 다음 tick 에서 실측 TF 로 확인한다.
        else:
            self.attempt_failed(f'{name} ended with status={status}')

    def _cancel_goal(self):
        self.generation += 1
        handle, self.handle = self.handle, None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.ctx.logger.warning(f'Cancelling the home goal failed: {exc}')

    # ------------------------------------------------------------ 결과
    def attempt_failed(self, message):
        ctx = self.ctx
        p = ctx.params
        self.attempts += 1
        ctx.state.error = message
        if self.attempts >= p.max_failures:
            self.fail(f'{message}. Retry {self.attempts}/{p.max_failures}; '
                      'send the switch again or call ~/resume')
            return
        ctx.logger.warning(f'Return home failed: {message}. Retry {self.attempts}/{p.max_failures}.')
        self.phase, self.retry_at = RETRY_WAIT, time.monotonic() + p.retry_delay_sec

    def fail(self, message):
        s = self.ctx.state
        self._cancel_goal()
        self.phase, s.state, s.error = FAILED, 'return_failed', message
        self.ctx.logger.error(message)
        self.publish_switch(False)

    def done(self, distance, yaw_error):
        s = self.ctx.state
        self.phase, s.state, s.error = DONE, 'returned_home', ''
        self.ctx.logger.info(f'Returned home: position error={distance:.3f}m, '
                             f'yaw error={math.degrees(yaw_error):.1f}deg')
        self.ctx.status.publish_status()
        self.publish_switch(False)  # 완료를 실측 TF 로 확인했으므로 스위치를 내린다.
