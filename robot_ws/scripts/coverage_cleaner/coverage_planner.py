"""왕복 sweep + A* 연결 + 누락 셀 보충으로 커버리지 경로 생성 (ROS 불필요).

별도 프로세스/스레드에서 호출해도 되도록 입력 배열만 받는 순수 함수다.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.ndimage import binary_dilation

from coverage_types import MapMeta, Pixel, Plan, PlanningError
from grid_ops import disk, inside, intervals, stamp
from path_search import astar, simplify
from parameters.thresholds import THRESHOLD_DEFAULTS
from traversability import reachable_map


def plan_coverage(safe: np.ndarray, free: np.ndarray, meta: MapMeta, start: Pixel,
                  visited: np.ndarray,
                  coverage_width: float = THRESHOLD_DEFAULTS['coverage_width'],
                  lane_spacing: float = THRESHOLD_DEFAULTS['lane_spacing'],
                  max_expansions: int = THRESHOLD_DEFAULTS['max_astar_expansions']) -> Plan:
    if min(coverage_width, lane_spacing, meta.resolution) <= 0:
        raise ValueError('coverage_width, lane_spacing and resolution must be positive')
    reachable = reachable_map(safe, start)
    kernel = disk(coverage_width / (2 * meta.resolution))
    target = free & binary_dilation(reachable, structure=kernel)
    predicted = visited.copy()
    route = [start]
    stamp(predicted, start, kernel)
    r = kernel.shape[0] // 2

    def append(goal):
        segment = astar(reachable, route[-1], goal, max_expansions)
        for p in segment[1:]:
            route.append(p)
            stamp(predicted, p, kernel)

    ys = np.flatnonzero(reachable.any(axis=1))
    step = max(1, int(math.floor(min(lane_spacing, coverage_width) / meta.resolution + 1e-9)))
    rows = list(range(int(ys[0]), int(ys[-1]) + 1, step))
    if rows[-1] != ys[-1]:
        rows.append(int(ys[-1]))
    if abs(start[0] - rows[-1]) < abs(start[0] - rows[0]):
        rows.reverse()
    left_to_right = start[1] <= np.flatnonzero(reachable[rows[0]]).mean()
    for y in rows:
        runs = intervals(reachable[y])
        if not left_to_right:
            runs.reverse()
        for lo, hi in runs:
            patch = np.s_[max(0, y - r):y + r + 1, max(0, lo - r):hi + r + 1]
            if not np.any(target[patch] & ~predicted[patch]):
                continue
            a, b = (int(lo), int(hi)) if left_to_right else (int(hi), int(lo))
            append((y, a))
            append((y, b))
        left_to_right = not left_to_right

    # A sweep alone misses short branches, concave boundaries and row remainders.
    while True:
        missing = np.argwhere(target & ~predicted)
        if not len(missing):
            break
        p = missing[np.argmin(np.sum((missing - np.asarray(route[-1])) ** 2, axis=1))]
        choices = []
        for dy, dx in np.argwhere(kernel) - r:
            q = int(p[0] + dy), int(p[1] + dx)
            if inside(reachable, q):
                choices.append(q)
        if not choices:
            raise PlanningError('A target has no reachable covering centre')
        goal = min(choices, key=lambda q: (q[0] - route[-1][0]) ** 2 + (q[1] - route[-1][1]) ** 2)
        append(goal)
        if not predicted[tuple(p)]:
            raise PlanningError('Coverage repair did not make progress')
    return Plan(simplify(route), target, predicted & free, reachable)
