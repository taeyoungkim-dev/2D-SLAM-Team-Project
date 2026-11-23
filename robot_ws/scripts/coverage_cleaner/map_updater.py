"""/explore_map 수신 처리: free/safe/plan_safe 지도 갱신과 tracker 재기준화.

장애물 구역(obstacle_zones)은 free 에서 빼서 "계획 영역"과 청소 대상에서만 제외한다.
  s.map_free   지도 그대로의 free
  s.free       map_free - zone                (청소 대상/방문 기록)
  s.safe       map_free 기준 정지 안전 영역    (로봇 위치 검사. zone 안에 있어도 물리적으로 안전하면 OK)
  s.plan_safe  s.free 기준 계획 영역           (zone 반지름 + 로봇 반지름 + 여유 + 계획 여유가 빠진다)
zone 이 활성 경로를 지나게 되면 이동을 취소하고 계획을 다시 만든다.
"""
from __future__ import annotations

import numpy as np

from coverage_tracker import CoverageTracker
from grid_ops import inside
from line_safety import line_is_safe
from map_input import parse_occupancy_grid
from traversability import traversable_map


class MapUpdater:
    def __init__(self, ctx):
        self.ctx = ctx

    def on_map(self, msg):
        ctx = self.ctx
        p = ctx.params
        try:
            meta, free = parse_occupancy_grid(msg, p.occupied_threshold)
        except (ValueError, TypeError) as exc:
            ctx.warn('bad_map', f'Ignoring invalid map: {exc}')
            return
        self._apply(meta, msg.info, free, 'map_changed')
        if ctx.obstacles.zones():
            ctx.status.publish_zone_map()   # 새 지도 격자에 맞춰 구역 지도를 다시 발행

    def on_zones_changed(self):
        """장애물 구역이 바뀌었을 때: 같은 지도로 마스크를 다시 만들고 경로를 검증한다."""
        ctx = self.ctx
        s = ctx.state
        if s.map_free is not None:
            self._apply(s.meta, s.info, s.map_free, 'obstacle_zone')
            ctx.status.publish_zone_map()
        ctx.home.on_zone_change()

    def _apply(self, meta, info, raw_free, reason):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        zone = ctx.obstacles.mask(meta)
        free = raw_free & ~zone if zone is not None else raw_free
        if (s.meta == meta and s.map_free is not None and np.array_equal(raw_free, s.map_free)
                and np.array_equal(free, s.free)):
            return
        old_meta = s.meta
        s.meta, s.info, s.map_free, s.free = meta, info, raw_free, free
        s.zone = zone
        res = meta.resolution
        # 정지 안전 영역은 장애물 구역과 무관하다(zone 안에 있어도 벽에서 멀면 안전).
        s.safe = traversable_map(raw_free, res, p.robot_radius, p.coverage_clearance)
        # Keep the runtime stop boundary independent of the planning margin.
        plan_clearance = p.coverage_clearance + p.planning_extra_margin
        s.plan_safe = traversable_map(free, res, p.robot_radius, plan_clearance)
        if zone is not None and zone.any():
            s.zone_stop = s.safe & ~traversable_map(free, res, p.robot_radius, p.coverage_clearance)
            s.zone_cut = traversable_map(raw_free, res, p.robot_radius, plan_clearance) & ~s.plan_safe
        else:
            s.zone_stop = s.zone_cut = None
        s.map_version += 1
        s.target = None
        if s.tracker is None:
            s.tracker = CoverageTracker(meta, p.coverage_width)
        elif not s.tracker.rebase(meta):
            ctx.warn('rebase', 'Grid geometry/frame changed incompatibly; visitation reset')
        s.tracker.visited &= free
        s.pass_start_count = None  # Different map denominators cannot be compared.
        if old_meta is not None and old_meta != meta:
            s.legs.clear()
            s.pass_in_progress = False
            ctx.executor.cancel(reason)
        elif s.active_leg and (
                not line_is_safe(*s.active_leg, meta, ctx.area.execution_mask())
                or not inside(s.plan_safe if p.check_safe_position else s.free,
                              meta.pixel(*s.active_leg[1]))
                or (not s.active_escape and ctx.area.leg_hits_zone(*s.active_leg))):
            if reason == 'obstacle_zone':
                ctx.logger.warning('Active path enters an obstacle zone: stopping and replanning')
            s.legs.clear()
            s.pass_in_progress = False
            ctx.executor.cancel(reason)
        elif any(not line_is_safe(a, b, meta, s.plan_safe)
                 for a, b in s.legs if (a, b) not in s.escape_legs):
            # Current segment remains safe. Replan after its result.
            s.legs.clear()
            s.pass_in_progress = False
