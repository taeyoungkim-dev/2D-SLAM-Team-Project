"""홈 복귀용 자체 경로 계획: 청소 불가 구역을 피하는 A* (ROS 불필요).

마스크 3개를 쓴다.
  plan_safe : zone 이 제외된 계획 영역(벽/장애물 여유 + zone 반지름 + 계획 여유 포함)
  escape    : zone 을 무시한 정지 안전 영역. 로봇이 이미 zone 안에 있을 때 빠져나오는 데만 쓴다.
  goal_safe : zone 을 반영한 정지 안전 영역. 홈이 계획 영역 밖(벽 여유 안쪽)일 때 마지막 연결에 쓴다.
"""
from __future__ import annotations

import numpy as np

from coverage_types import Pixel, PlanningError
from grid_ops import inside
from path_search import astar, simplify
from traversability import reachable_map


def nearest(candidates: np.ndarray, target: Pixel) -> Pixel:
    pts = np.argwhere(candidates)
    if not len(pts):
        raise PlanningError('No planning cell is reachable')
    d = np.sum((pts - np.asarray(target)) ** 2, axis=1)
    return tuple(int(v) for v in pts[int(np.argmin(d))])


def plan_route(plan_safe: np.ndarray, escape: np.ndarray, goal_safe: np.ndarray,
               start: Pixel, goal: Pixel, max_expansions: int) -> list[Pixel]:
    """start -> goal 픽셀 경로(단순화됨). 계획할 수 없으면 PlanningError."""
    head: list[Pixel] = []
    if inside(plan_safe, start):
        a = start
    else:
        a = nearest(plan_safe & reachable_map(escape, start), start)
        head = astar(escape, start, a, max_expansions)[:-1]
    tail: list[Pixel] = []
    if inside(plan_safe, goal):
        b = goal
    else:
        if not inside(goal_safe, goal):
            raise PlanningError('The home position is inside an obstacle zone or outside the safe map')
        b = nearest(plan_safe & reachable_map(plan_safe, a), goal)
        tail = astar(goal_safe, b, goal, max_expansions)[1:]
    middle = astar(plan_safe, a, b, max_expansions)
    return simplify(head + middle + tail)
