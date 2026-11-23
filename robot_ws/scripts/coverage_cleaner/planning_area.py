"""계획 시작 셀/계획 영역/청소 대상 셀 결정."""
from __future__ import annotations

import numpy as np
from scipy.ndimage import binary_dilation

from coverage_types import PlanningError, PlanningMarginError
from grid_ops import disk, inside
from traversability import reachable_map


class PlanningArea:
    def __init__(self, ctx):
        self.ctx = ctx

    def planning_start(self, pose):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        start = s.meta.pixel(*pose)
        if not p.check_safe_position and not inside(s.plan_safe, start):
            # The coverage sweep keeps its original clearance. Only the entry
            # connector may use known-free cells inside that clearance margin.
            reachable_free = reachable_map(s.free, start)
            candidates = np.argwhere(s.plan_safe & reachable_free)
            if not len(candidates):
                raise PlanningError('No planning area is reachable through known-free cells')
            distances = np.sum((candidates - np.asarray(start)) ** 2, axis=1)
            anchor = tuple(int(v) for v in candidates[int(np.argmin(distances))])
            ctx.warn('relaxed_planning_start',
                     f'check_safe_position=False: robot_xy={pose}; '
                     f'planning_anchor_xy={s.meta.world(*anchor)}; '
                     'entry connector uses known-free cells without footprint clearance')
            return anchor
        if not inside(s.safe, start):
            raise PlanningError('Robot is outside the safe map. Check TF, footprint and clearance.')
        if not inside(s.plan_safe, start):
            raise PlanningMarginError(
                f'Robot is stop-safe but outside the planning area: robot_xy={pose}; '
                f'grid_cell_yx={start}; extra_margin={p.planning_extra_margin:.3f}m. '
                'Place the robot farther from obstacles before planning.')
        return start

    def execution_mask(self):
        """Disabling clearance checks still preserves known-free path geometry."""
        s = self.ctx.state
        return s.safe if self.ctx.params.check_safe_position else s.free

    def wait_for_planning_area(self, message):
        ctx = self.ctx
        s = ctx.state
        ctx.executor.cancel('planning_margin')
        s.legs.clear()
        s.pass_in_progress = False
        s.plan_version = -1
        s.pass_start_count = None
        s.state = 'waiting_for_planning_area'
        ctx.warn('planning_margin', str(message))
        ctx.status.publish_status()

    def refresh_target(self, pose):
        # Coverage targets and the repair sweep must use the same area as
        # the planner, so they cannot pull the path back towards a wall.
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        reachable = reachable_map(s.plan_safe, self.planning_start(pose))
        kernel = disk(p.coverage_width / (2 * s.meta.resolution))
        s.target = s.free & binary_dilation(reachable, structure=kernel)
