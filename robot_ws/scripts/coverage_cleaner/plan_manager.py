"""커버리지 계획 요청(백그라운드 스레드)과 결과 수락."""
from __future__ import annotations

import math
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from coverage_planner import plan_coverage
from coverage_types import PlanningError, PlanningMarginError
from leg_splitter import split_legs
from line_safety import line_is_safe
from path_message import build_path_message
from path_search import astar, simplify


class PlanManager:
    def __init__(self, ctx, path_pub):
        self.ctx = ctx
        self.path_pub = path_pub
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='coverage-plan')

    def start_plan(self, pose):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        try:
            start = ctx.area.planning_start(pose)
        except PlanningMarginError as exc:
            ctx.area.wait_for_planning_area(exc)
            return
        except PlanningError as exc:
            s.state = 'waiting_for_safe_pose'
            ctx.warn('planning_start', str(exc))
            return
        s.state = 'planning'
        s.plan_version = s.map_version
        s.plan_future = self.pool.submit(plan_coverage, s.plan_safe.copy(), s.free.copy(), s.meta,
                                         start, s.tracker.visited.copy(),
                                         p.coverage_width, p.lane_spacing, p.max_astar_expansions)

    def collect_result(self, pose):
        """진행 중인 계획 결과를 수거한다. tick 이 이어서 진행해도 되면 True."""
        ctx = self.ctx
        s = ctx.state
        if s.plan_future is not None:
            if not s.plan_future.done():
                return False
            future, s.plan_future = s.plan_future, None
            if s.plan_version != s.map_version:
                return False  # A map change invalidates this snapshot.
            try:
                plan = future.result()
            except Exception as exc:
                ctx.block(f'Coverage planning failed: {exc}')
                return False
            if not self.accept_plan(plan, pose):
                return False
        return True

    def accept_plan(self, plan, pose):
        ctx = self.ctx
        s, p = ctx.state, ctx.params
        try:
            ctx.area.planning_start(pose)
        except PlanningMarginError as exc:
            ctx.area.wait_for_planning_area(exc)
            return False
        except PlanningError as exc:
            s.state = 'waiting_for_safe_pose'
            ctx.warn('planning_start', str(exc))
            return False
        world = [s.meta.world(*px) for px in plan.pixels]
        escape_part = None
        if ctx.area.in_zone_cut(pose):
            # 로봇이 장애물 구역 안에 있다: 구역을 무시한 안전 영역으로 계획 시작 셀까지 빠져나온다.
            try:
                connector = simplify(astar(s.safe, s.meta.pixel(*pose), plan.pixels[0],
                                           p.max_astar_expansions))
            except PlanningError as exc:
                ctx.block(f'Cannot leave the obstacle zone: {exc}')
                return False
            middle = [s.meta.world(*px) for px in connector[1:-1]]
            escape_part = [pose] + middle + [world[0]]
            world = [pose] + middle + world
        elif not p.check_safe_position:
            try:
                connector = astar(s.free, s.meta.pixel(*pose), plan.pixels[0],
                                  p.max_astar_expansions)
            except PlanningError as exc:
                ctx.block(f'Cannot connect to the coverage plan through known-free cells: {exc}')
                return False
            world = [s.meta.world(*px) for px in connector[:-1]] + world
        elif not line_is_safe(pose, world[0], s.meta, s.plan_safe):
            ctx.block('Cannot safely connect the actual robot pose to the planned first cell')
            return False
        if math.dist(pose, world[0]) > 1e-9:
            world.insert(0, pose)
        self.path_pub.publish(build_path_message(world, s.meta, ctx.clock, p.path_resolution))
        s.preview_version = s.map_version
        if not p.send_to_nav2:
            s.state = 'preview'
            return True
        s.legs = deque(split_legs(world, p.max_leg_length))
        s.escape_legs = set(split_legs(escape_part, p.max_leg_length)) if escape_part else set()
        s.pass_in_progress = True
        s.pass_start_count = int(np.count_nonzero(s.target & s.tracker.visited))
        s.state = 'executing'
        ctx.logger.info(f'Plan: {len(plan.pixels)} corners, {len(s.legs)} FollowPath segments; '
                        f'remaining={np.count_nonzero(s.target & ~s.tracker.visited)} cells')
        return True

    def shutdown(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
