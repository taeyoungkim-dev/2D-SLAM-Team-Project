"""/explore_map 수신 처리: free/safe/plan_safe 지도 갱신과 tracker 재기준화."""
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
        s, p = ctx.state, ctx.params
        try:
            meta, free = parse_occupancy_grid(msg, p.occupied_threshold)
        except (ValueError, TypeError) as exc:
            ctx.warn('bad_map', f'Ignoring invalid map: {exc}')
            return
        if s.meta == meta and np.array_equal(free, s.free):
            return
        old_meta = s.meta
        s.meta, s.info, s.free = meta, msg.info, free
        s.safe = traversable_map(free, meta.resolution, p.robot_radius, p.coverage_clearance)
        # Keep the runtime stop boundary independent of the planning margin.
        s.plan_safe = traversable_map(
            free, meta.resolution, p.robot_radius,
            p.coverage_clearance + p.planning_extra_margin)
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
            ctx.executor.cancel('map_changed')
        elif s.active_leg and (
                not line_is_safe(*s.active_leg, meta, ctx.area.execution_mask())
                or not inside(s.plan_safe if p.check_safe_position else s.free,
                              meta.pixel(*s.active_leg[1]))):
            s.legs.clear()
            s.pass_in_progress = False
            ctx.executor.cancel('map_changed')
        elif any(not line_is_safe(a, b, meta, s.plan_safe) for a, b in s.legs):
            # Current segment remains safe. Replan after its result.
            s.legs.clear()
            s.pass_in_progress = False
