"""구간 큐가 비었을 때: 경로 완료/전체 완료/무진전 판정 후 재계획."""
from __future__ import annotations

import numpy as np


class PassCompletion:
    def __init__(self, ctx):
        self.ctx = ctx

    def evaluate(self, pose):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        remaining = int(np.count_nonzero(s.target & ~s.tracker.visited))
        # An empty queue alone is insufficient: failures, cancellations and
        # map/TF invalidation also clear it. Only a valid, drained pass ends
        # single-pass cleaning; recovery plans use the existing retry flow.
        if s.pass_in_progress and not p.reclean_unvisited:
            s.pass_in_progress = False
            s.single_pass_finished = True
            s.pass_start_count = None
            s.state = 'complete_pass' if remaining else 'complete_reachable'
            s.error = ''
            ctx.logger.info(
                f'Coverage path complete; reclean_unvisited=False; '
                f'remaining_target_cells={remaining}; additional cleanup disabled. '
                'Visited records retained; call ~/resume to start another pass.')
            ctx.status.publish_debug()
            ctx.home.on_cleaning_complete()
            return
        s.pass_in_progress = False
        if remaining == 0:
            first = s.state != 'complete_reachable'
            if first:
                unreachable = np.count_nonzero(s.free & ~s.target & ~s.tracker.visited)
                ctx.logger.info('Reachable coverage complete (measured TF); '
                                f'{unreachable} known-free cells remain outside the reachable target')
            s.state = 'complete_reachable'
            s.error = ''
            ctx.status.publish_status()
            if first:  # evaluate() 는 완료 상태에서 매 tick 호출되므로 처음 한 번만 켠다.
                ctx.home.on_cleaning_complete()
            return
        done = int(np.count_nonzero(s.target & s.tracker.visited))
        if s.pass_start_count is not None:
            if done <= s.pass_start_count:
                s.no_progress_passes += 1
            else:
                s.no_progress_passes = 0
                s.failures = 0
            s.pass_start_count = None
            if s.no_progress_passes >= p.max_no_progress_passes:
                ctx.block('Unvisited cells remain after repeated passes. Check goal tolerance, '
                          'localization, controller tracking and coverage_width; use ~/resume after fixing.')
                return
        ctx.planner.start_plan(pose)
