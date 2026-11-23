"""컴포넌트 공용 컨텍스트: params + state + logger + clock + 컴포넌트 참조.

cleaner_node.py 가 컴포넌트를 만든 뒤 아래 속성에 연결한다.
  pose_provider, map_updater, area, planner, goal_checker, executor,
  status, completion, home
컴포넌트끼리는 ctx.<이름>.<메서드> 로 서로를 호출한다.
"""
from __future__ import annotations

import math
import time

from parameters.thresholds import WARN_THROTTLE_SEC


class Context:
    def __init__(self, params, state, logger, clock):
        self.params, self.state, self.logger, self.clock = params, state, logger, clock
        self.pose_provider = self.map_updater = self.area = self.planner = None
        self.goal_checker = self.executor = self.status = self.completion = None
        self.home = None

    def warn(self, key, message):
        now = time.monotonic()
        if now - self.state.last_warning.get(key, -math.inf) >= WARN_THROTTLE_SEC:
            self.logger.warning(message)
            self.state.last_warning[key] = now

    def pause(self, reason):
        """청소를 멈추고 진행 중인 FollowPath 를 취소한다(재개는 ~/resume)."""
        s = self.state
        s.enabled = False
        s.legs.clear()
        s.pass_in_progress = False
        s.plan_version = -1
        if s.tracker:
            s.tracker.previous = None
        self.executor.cancel(reason)

    def block(self, message):
        s = self.state
        s.halted, s.error, s.state = True, message, 'blocked'
        s.legs.clear()
        s.pass_in_progress = False
        self.executor.cancel('blocked')
        self.logger.error(message)
